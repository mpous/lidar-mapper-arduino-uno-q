#!/usr/bin/env python3
"""
LiDAR Mapper – MPU server for Arduino UNO Q
============================================
Reads LDROBOT D500 LiDAR directly from its USB serial port, accumulates a
full 360° scan, and streams it to any connected browser via WebSocket.

The D500 is connected via USB (appears as /dev/ttyUSB0 or /dev/ttyACM0).

Access the map at: http://<board-ip>:5001
"""

import glob
import threading
import time

from flask import Flask, render_template
from flask_socketio import SocketIO

# ── Flask / SocketIO setup ────────────────────────────────────────────────────
app = Flask(__name__, template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Scan state ────────────────────────────────────────────────────────────────
scan_buffer: dict[int, int] = {}
scan_lock = threading.Lock()
last_end_angle: float = -1.0
scan_count: int = 0

# ── D500 protocol constants ───────────────────────────────────────────────────
_HDR1    = 0x54
_HDR2    = 0x2C
_PKT_LEN = 47
_POINTS  = 12


def _crc8(data: bytes | bytearray) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x4D) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


def _process_packet(buf: bytearray) -> None:
    global last_end_angle, scan_count

    if _crc8(buf[:_PKT_LEN - 1]) != buf[_PKT_LEN - 1]:
        return  # CRC mismatch

    sa_cdeg = (buf[5] << 8) | buf[4]   # start angle ×100 °
    ea_cdeg = (buf[43] << 8) | buf[42] # end angle   ×100 °
    sa_deg  = sa_cdeg / 100.0
    ea_deg  = ea_cdeg / 100.0

    is_new_revolution = last_end_angle > 180.0 and sa_deg < 90.0

    with scan_lock:
        for i in range(_POINTS):
            base = 6 + i * 3
            dist = (buf[base + 1] << 8) | buf[base]  # mm
            if dist == 0 or dist > 12000:
                continue
            angle = sa_deg + (ea_deg - sa_deg) * i / (_POINTS - 1)
            scan_buffer[int(angle) % 360] = dist

        if is_new_revolution and scan_buffer:
            scan_count += 1

    last_end_angle = ea_deg


def _find_lidar_port() -> str:
    """Return the first available USB serial device, defaulting to /dev/ttyUSB0."""
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    return candidates[0] if candidates else "/dev/ttyUSB0"


def _lidar_reader() -> None:
    """Background thread: opens USB serial, parses D500 packets indefinitely."""
    import serial  # imported here so the module loads even without pyserial installed

    port = _find_lidar_port()
    print(f"[LiDAR] Connecting to {port} at 230400 baud …")

    while True:
        try:
            with serial.Serial(port, 230400, timeout=1) as ser:
                print(f"[LiDAR] Connected to {port}")
                state = "WAIT_H1"
                buf   = bytearray()

                while True:
                    raw = ser.read(1)
                    if not raw:
                        continue
                    byte = raw[0]

                    if state == "WAIT_H1":
                        if byte == _HDR1:
                            buf   = bytearray([byte])
                            state = "WAIT_H2"

                    elif state == "WAIT_H2":
                        if byte == _HDR2:
                            buf.append(byte)
                            state = "READ_BODY"
                        elif byte == _HDR1:
                            buf = bytearray([byte])   # false start, re-sync
                        else:
                            state = "WAIT_H1"

                    elif state == "READ_BODY":
                        buf.append(byte)
                        if len(buf) == _PKT_LEN:
                            _process_packet(buf)
                            state = "WAIT_H1"

        except Exception as exc:
            print(f"[LiDAR] {exc} – retrying in 2 s")
            time.sleep(2)


# ── Start LiDAR reader thread ─────────────────────────────────────────────────
lidar_thread = threading.Thread(target=_lidar_reader, daemon=True)
lidar_thread.start()


# ── Background broadcaster ────────────────────────────────────────────────────
def _broadcast_scan() -> None:
    """Emit the current scan buffer to all WebSocket clients at ~10 Hz."""
    while True:
        time.sleep(0.1)
        with scan_lock:
            if not scan_buffer:
                continue
            points   = [[angle, dist] for angle, dist in sorted(scan_buffer.items())]
            stats    = {"count": len(points), "scan_num": scan_count}
        socketio.emit("scan", {"points": points, "stats": stats})


broadcast_thread = threading.Thread(target=_broadcast_scan, daemon=True)
broadcast_thread.start()


# ── Routes ────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/health")
def health():
    with scan_lock:
        return {"status": "ok", "points": len(scan_buffer), "scans": scan_count}


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("LiDAR Mapper running on http://0.0.0.0:5001")
    socketio.run(app, host="0.0.0.0", port=5001, allow_unsafe_werkzeug=True)
