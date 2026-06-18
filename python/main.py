#!/usr/bin/env python3
"""
LiDAR Mapper – MPU server for Arduino UNO Q
============================================
Reads LDROBOT D500 LiDAR via USB serial, displays live 360-degree map,
and supports Edge Impulse training data forwarding and model inference.

Access the map at: http://<board-ip>:5001
"""

import glob
import os
import threading
import time

import requests as _http
from flask import Flask, render_template
from flask_socketio import SocketIO

try:
    from edge_impulse_linux.runner import ImpulseRunner
    _EI_RUNNER_AVAILABLE = True
except ImportError:
    _EI_RUNNER_AVAILABLE = False

# ── Flask / SocketIO setup ────────────────────────────────────────────────────
app = Flask(__name__, template_folder="templates")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ── Scan state ────────────────────────────────────────────────────────────────
scan_buffer: dict[int, int] = {}
scan_lock = threading.Lock()
last_end_angle: float = -1.0
scan_count: int = 0

# ── LiDAR connection state ───────────────────────────────────────────────────
_lidar_connected = False
_lidar_port = ""
_lidar_stop = threading.Event()
_lidar_thread: threading.Thread | None = None

# ── Training mode (Edge Impulse data forwarding) ─────────────────────────────
_training_lock = threading.Lock()
_training_active = False
_training_api_key = ""
_training_label = ""
_training_samples: list[list[int]] = []
_training_start_time = 0.0
_training_duration_s = 10

_EI_INGESTION_URL = "https://ingestion.edgeimpulse.com/api/training/data"

# ── Inference mode (Edge Impulse model) ──────────────────────────────────────
_inference_lock = threading.Lock()
_inference_active = False
_inference_runner = None
_inference_n_scans = 1
_inference_buffer: list[list[int]] = []
_inference_model_info: dict = {}
_inference_last_classify = 0.0
_CLASSIFY_INTERVAL = 0.2

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
        return

    sa_cdeg = (buf[5] << 8) | buf[4]
    ea_cdeg = (buf[43] << 8) | buf[42]
    sa_deg  = sa_cdeg / 100.0
    ea_deg  = ea_cdeg / 100.0

    is_new_revolution = last_end_angle > 180.0 and sa_deg < 90.0

    with scan_lock:
        for i in range(_POINTS):
            base = 6 + i * 3
            dist = (buf[base + 1] << 8) | buf[base]
            if dist == 0 or dist > 12000:
                continue
            angle = sa_deg + (ea_deg - sa_deg) * i / (_POINTS - 1)
            scan_buffer[int(angle) % 360] = dist

        if is_new_revolution and scan_buffer:
            scan_count += 1

    last_end_angle = ea_deg


# ── Serial port helpers ──────────────────────────────────────────────────────

def _list_serial_ports() -> list[dict]:
    ports: list[dict] = []
    try:
        import serial.tools.list_ports
        for p in serial.tools.list_ports.comports():
            ports.append({"device": p.device, "description": p.description or p.device})
    except Exception:
        pass
    for path in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if not any(pp["device"] == path for pp in ports):
            ports.append({"device": path, "description": path})
    return ports


def _find_lidar_port() -> str:
    env_port = os.environ.get("LIDAR_PORT")
    if env_port and os.path.exists(env_port):
        print(f"[LIDAR] Using LIDAR_PORT env var: {env_port}")
        return env_port

    # Diagnostic: show what tty/serial devices exist in /dev
    print("[LIDAR] Searching for Lidar port in /dev/...")
    try:
        all_dev = os.listdir("/dev")
        tty_devs = sorted(d for d in all_dev if d.startswith("tty"))
        print(f"[LIDAR] /dev/tty* devices visible: {tty_devs}")
        serial_exists = os.path.isdir("/dev/serial")
        print(f"[LIDAR] /dev/serial/ exists: {serial_exists}")
        if serial_exists:
            import pathlib
            serial_devs = list(pathlib.Path("/dev/serial").rglob("*"))
            print(f"[LIDAR] /dev/serial/ contents: {serial_devs}")
    except Exception as exc:
        print(f"[LIDAR] Diagnostic listing failed: {exc}")

    for i in range(10):
        candidates = sorted(
            glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/serial/by-id/*")
        )
        if candidates:
            print(f"[LIDAR] Found candidates: {candidates}")
            for candidate in candidates:
                if os.path.exists(candidate):
                    print(f"[LIDAR] Verified candidate: {candidate}")
                    return candidate
        else:
            if i == 0:
                print(f"[LIDAR] No /dev/ttyUSB* or /dev/ttyACM* found — device may not be mapped into container")
            print(f"[LIDAR] No candidates found in iteration {i+1}/10")
        time.sleep(1)

    print("[LIDAR] No Lidar port found.")
    print("[LIDAR] If running via arduino-app-cli, try running directly instead:")
    print("[LIDAR]   cd /home/arduino/ArduinoApps/lidar-mapper")
    print("[LIDAR]   source .venv/bin/activate")
    print("[LIDAR]   python3 python/main.py")
    return "/dev/ttyUSB0"


# ── LiDAR reader (stoppable / restartable) ───────────────────────────────────

def _lidar_reader(port: str) -> None:
    global _lidar_connected
    import serial

    print(f"[LiDAR] Starting on {port} at 230400 baud …")

    while not _lidar_stop.is_set():
        try:
            with serial.Serial(port, 230400, timeout=1) as ser:
                _lidar_connected = True
                socketio.emit("lidar_status", {"connected": True, "port": port, "running": True})
                print(f"[LiDAR] Connected to {port}")
                state = "WAIT_H1"
                buf   = bytearray()

                while not _lidar_stop.is_set():
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
                            buf = bytearray([byte])
                        else:
                            state = "WAIT_H1"

                    elif state == "READ_BODY":
                        buf.append(byte)
                        if len(buf) == _PKT_LEN:
                            _process_packet(buf)
                            state = "WAIT_H1"

        except Exception as exc:
            _lidar_connected = False
            socketio.emit("lidar_status", {
                "connected": False, "port": port,
                "error": str(exc), "running": not _lidar_stop.is_set(),
            })
            print(f"[LiDAR] {exc} – retrying in 2 s")
            for _ in range(20):
                if _lidar_stop.is_set():
                    break
                time.sleep(0.1)

    _lidar_connected = False
    socketio.emit("lidar_status", {"connected": False, "port": port, "running": False})
    print("[LiDAR] Stopped")


def _start_lidar(port: str) -> None:
    global _lidar_thread, _lidar_port
    _stop_lidar()
    if not port:
        socketio.emit("lidar_status", {
            "connected": False, "port": "", "running": False,
            "error": "No port selected",
        })
        return
    _lidar_stop.clear()
    _lidar_port = port
    _lidar_thread = threading.Thread(target=_lidar_reader, args=(port,), daemon=True)
    _lidar_thread.start()


def _stop_lidar() -> None:
    global _lidar_connected
    _lidar_stop.set()
    if _lidar_thread and _lidar_thread.is_alive():
        _lidar_thread.join(timeout=3)
    _lidar_connected = False


# Auto-start with detected port
_auto_port = _find_lidar_port()
if _auto_port:
    _start_lidar(_auto_port)


# ── Background broadcaster ────────────────────────────────────────────────────
def _broadcast_scan() -> None:
    global _inference_last_classify

    while True:
        time.sleep(0.1)
        with scan_lock:
            if not scan_buffer:
                continue
            points   = [[a, d] for a, d in sorted(scan_buffer.items())]
            stats    = {"count": len(points), "scan_num": scan_count}
            snapshot = dict(scan_buffer)
        socketio.emit("scan", {"points": points, "stats": stats})

        # Training data capture
        should_finish = False
        with _training_lock:
            if _training_active:
                elapsed = time.time() - _training_start_time
                if elapsed >= _training_duration_s:
                    should_finish = True
                else:
                    row = [snapshot.get(deg, 0) for deg in range(360)]
                    _training_samples.append(row)
                    progress = min(elapsed / _training_duration_s * 100, 100)
                    socketio.emit("training_progress", {
                        "progress": round(progress, 1),
                        "samples": len(_training_samples),
                    })
        if should_finish:
            _finish_training()

        # Inference classification
        with _inference_lock:
            if _inference_active and _inference_runner:
                row = [snapshot.get(deg, 0) for deg in range(360)]
                _inference_buffer.append(row)
                n = _inference_n_scans
                if len(_inference_buffer) > n:
                    _inference_buffer[:] = _inference_buffer[-n:]
                now = time.time()
                if len(_inference_buffer) >= n and (now - _inference_last_classify) >= _CLASSIFY_INTERVAL:
                    _inference_last_classify = now
                    features: list[int] = []
                    for r in _inference_buffer:
                        features.extend(r)
                    expected = _inference_model_info.get(
                        "model_parameters", {},
                    ).get("input_features_count", len(features))
                    features = features[:expected]
                    while len(features) < expected:
                        features.append(0)
                    try:
                        res = _inference_runner.classify(features)
                        socketio.emit("inference_result", res)
                    except Exception as exc:
                        socketio.emit("inference_result", {"error": str(exc)})


broadcast_thread = threading.Thread(target=_broadcast_scan, daemon=True)
broadcast_thread.start()


# ── LiDAR control events ────────────────────────────────────────────────────

@socketio.on("list_ports")
def _on_list_ports(_data=None):
    ports = _list_serial_ports()
    socketio.emit("ports_list", {"ports": ports, "current": _lidar_port})


@socketio.on("lidar_start")
def _on_lidar_start(data):
    port = (data.get("port") or "").strip()
    if port:
        _start_lidar(port)


@socketio.on("lidar_stop")
def _on_lidar_stop(_data=None):
    _stop_lidar()
    socketio.emit("lidar_status", {"connected": False, "port": _lidar_port, "running": False})


@socketio.on("get_status")
def _on_get_status(_data=None):
    socketio.emit("app_status", {
        "lidar_connected": _lidar_connected,
        "lidar_port": _lidar_port,
        "lidar_running": not _lidar_stop.is_set(),
        "ei_runner_available": _EI_RUNNER_AVAILABLE,
        "inference_active": _inference_active,
        "training_active": _training_active,
    })
    ports = _list_serial_ports()
    socketio.emit("ports_list", {"ports": ports, "current": _lidar_port})


# ── Training mode events ────────────────────────────────────────────────────

@socketio.on("training_start")
def _on_training_start(data):
    global _training_active, _training_api_key, _training_label
    global _training_samples, _training_start_time, _training_duration_s

    api_key  = (data.get("api_key") or "").strip()
    label    = (data.get("label") or "").strip()
    duration = int(data.get("duration_s", 10))

    if not api_key:
        socketio.emit("training_status", {"state": "error", "message": "API key required"})
        return
    if not label:
        socketio.emit("training_status", {"state": "error", "message": "Label required"})
        return

    with _training_lock:
        _training_active    = True
        _training_api_key   = api_key
        _training_label     = label
        _training_duration_s = duration
        _training_samples   = []
        _training_start_time = time.time()

    print(f"[Training] Recording '{label}' for {duration}s")
    socketio.emit("training_status", {
        "state": "recording",
        "message": f"Recording '{label}' for {duration}s…",
    })


@socketio.on("training_stop")
def _on_training_stop(_data=None):
    _finish_training()


def _finish_training():
    global _training_active

    with _training_lock:
        if not _training_active:
            return
        _training_active = False
        samples  = list(_training_samples)
        api_key  = _training_api_key
        label    = _training_label
        _training_samples.clear()

    if not samples:
        socketio.emit("training_status", {"state": "error", "message": "No data collected"})
        return

    socketio.emit("training_status", {
        "state": "uploading",
        "message": f"Uploading {len(samples)} scans…",
    })
    threading.Thread(target=_upload_to_ei, args=(samples, api_key, label), daemon=True).start()


def _upload_to_ei(samples: list[list[int]], api_key: str, label: str) -> None:
    sensors = [{"name": f"d_{deg:03d}", "units": "mm"} for deg in range(360)]

    envelope = {
        "protected": {"ver": "v1", "alg": "none", "iat": int(time.time())},
        "signature": "0" * 64,
        "payload": {
            "device_name": "arduino-uno-q-lidar",
            "device_type": "LDROBOT_D500",
            "interval_ms": 100,
            "sensors": sensors,
            "values": samples,
        },
    }

    try:
        resp = _http.post(
            _EI_INGESTION_URL,
            headers={
                "x-api-key": api_key,
                "x-label": label,
                "Content-Type": "application/json",
                "x-file-name": f"lidar_{label}_{int(time.time())}",
            },
            json=envelope,
            timeout=30,
        )
        if resp.status_code == 200:
            print(f"[Training] Uploaded {len(samples)} scans as '{label}'")
            socketio.emit("training_status", {
                "state": "done",
                "message": f"Uploaded {len(samples)} scans as '{label}'",
            })
        else:
            print(f"[Training] Upload failed ({resp.status_code}): {resp.text[:200]}")
            socketio.emit("training_status", {
                "state": "error",
                "message": f"Upload failed ({resp.status_code}): {resp.text[:200]}",
            })
    except Exception as exc:
        print(f"[Training] Upload error: {exc}")
        socketio.emit("training_status", {
            "state": "error",
            "message": f"Upload error: {exc}",
        })


# ── Inference mode events ────────────────────────────────────────────────────

@socketio.on("inference_start")
def _on_inference_start(data):
    global _inference_active, _inference_runner, _inference_n_scans
    global _inference_buffer, _inference_model_info

    if not _EI_RUNNER_AVAILABLE:
        socketio.emit("inference_status", {
            "state": "error",
            "message": "edge_impulse_linux not installed. Run: pip install edge_impulse_linux",
        })
        return

    model_path = (data.get("model_path") or "").strip()
    if not model_path:
        socketio.emit("inference_status", {"state": "error", "message": "Model path required"})
        return
    if not os.path.isfile(model_path):
        socketio.emit("inference_status", {
            "state": "error",
            "message": f"Model file not found: {model_path}",
        })
        return

    socketio.emit("inference_status", {"state": "loading", "message": "Loading model…"})

    def _load():
        global _inference_active, _inference_runner, _inference_n_scans
        global _inference_buffer, _inference_model_info
        try:
            runner = ImpulseRunner(model_path)
            model_info = runner.init()

            features_count = model_info.get("model_parameters", {}).get(
                "input_features_count", 360,
            )
            n_scans = max(features_count // 360, 1)

            with _inference_lock:
                if _inference_runner:
                    try:
                        _inference_runner.stop()
                    except Exception:
                        pass
                _inference_runner = runner
                _inference_model_info = model_info
                _inference_n_scans = n_scans
                _inference_buffer = []
                _inference_active = True

            labels = model_info.get("model_parameters", {}).get("labels", [])
            print(f"[Inference] Model loaded: {labels}, window={n_scans} scans")
            socketio.emit("inference_status", {
                "state": "running",
                "message": f"Model loaded — {len(labels)} classes, window {n_scans} scan(s)",
                "model_info": {
                    "labels": labels,
                    "input_features_count": features_count,
                    "window_scans": n_scans,
                },
            })
        except Exception as exc:
            print(f"[Inference] Load error: {exc}")
            socketio.emit("inference_status", {
                "state": "error",
                "message": f"Failed to load model: {exc}",
            })

    threading.Thread(target=_load, daemon=True).start()


@socketio.on("inference_stop")
def _on_inference_stop(_data=None):
    global _inference_active, _inference_runner

    with _inference_lock:
        _inference_active = False
        if _inference_runner:
            try:
                _inference_runner.stop()
            except Exception:
                pass
            _inference_runner = None
        _inference_buffer.clear()

    socketio.emit("inference_status", {"state": "stopped", "message": "Inference stopped"})
    print("[Inference] Stopped")


# ── Routes ────────────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/ports")
def api_ports():
    return {"ports": _list_serial_ports(), "current": _lidar_port}


@app.route("/health")
def health():
    with scan_lock:
        return {
            "status": "ok",
            "points": len(scan_buffer),
            "scans": scan_count,
            "lidar_connected": _lidar_connected,
            "ei_runner_available": _EI_RUNNER_AVAILABLE,
        }


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("LiDAR Mapper running on http://0.0.0.0:5001")
    socketio.run(app, host="0.0.0.0", port=5001, allow_unsafe_werkzeug=True)
