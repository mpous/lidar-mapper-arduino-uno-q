# LiDAR mapper for Arduino UNO Q

In this project we will connect an LDRobot D500 LiDAR Developer Kit to an Arduino UNO Q, capture distance profiles from different rooms, upload them to Edge Impulse Studio as time-series data, train a classification model, and deploy it back on the Arduino UNO Q for real-time inference, all happening in the edge. 

The Arduino UNO Q renders a local Web UI for real-time polar map and integrates with Edge Impulse, using the API key, for training data capture and on-device ML inference.

## How the LiDAR works

```
LDROBOT D500 ──USB──▶ MPU (Python / Flask)
                          │
                          ├── parses D500 binary protocol
                          ├── accumulates 360° scans
                          ├── serves web UI on port 5001
                          └── broadcasts scan data via WebSocket @ 10 Hz
```

## Hardware

- Arduino UNO Q board
- LDROBOT D500 LiDAR sensor connected via USB

## Installation & Deployment

### Copy the project to the Arduino UNO Q

```
scp -r . arduino@<board-ip>:/home/arduino/ArduinoApps/lidar-mapper/
```

### SSH into the board and set up the environment

```
ssh arduino@<board-ip>
cd /home/arduino/ArduinoApps/lidar-mapper
python3 -m venv .venv
source .venv/bin/activate
pip install --no-cache-dir -r python/requirements.txt
```

### Start the application

Using the Arduino App CLI:

```
arduino-app-cli app start .
```

Or run the Python server directly:

```
source .venv/bin/activate
python3 python/main.py
```

### Open the Web UI

Navigate to `http://<board-ip>:5001` in your browser.


## Edge Impulse Studio integration

### Capture training data

1. Open the **Training** tab in the web UI
2. Enter your Edge Impulse project **API key** (found in Dashboard > Keys)
3. Set a **label** (e.g., `obstacle`, `empty`, `wall`)
4. Choose a recording duration and click **Record**
5. Scans are uploaded to your Edge Impulse project as time-series data with 360 features (one distance per degree)

### Running model inference

1. Train a model in Edge Impulse Studio using the captured LiDAR data
2. Deploy as **Arduino UNO Q**
3. Install the runtime: `pip install edge_impulse_linux`
4. Open the **Inference** tab, enter the model path, and click **Start Inference**
5. Classification results appear as live confidence bars overlaid on the map

## Disclaimer

Use responsibly. This project is for entertainment purposes only. Do not use this code in production. Try it and improve it!


