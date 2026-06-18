# LiDAR mapper for Arduino UNO Q

In this project we will connect an LDRobot D500 LiDAR Developer Kit to an Arduino UNO Q, capture distance profiles from different rooms, upload them to Edge Impulse Studio as time-series data, train a classification model, and deploy it back on the Arduino UNO Q for real-time inference, all happening in the edge. 

<img width="800" alt="Mapping a room with the LiDAR connected to the Arduino UNO Q and Edge Impulse" src="https://github.com/user-attachments/assets/97fe280a-f3ed-4c3c-b2bf-db917a718854" />

The Arduino UNO Q renders a local Web UI for real-time polar map and integrates with Edge Impulse, using the API key, for training data capture and on-device ML inference.

## How the LiDAR works

LiDAR stands for Light Detection and Ranging. The LiDAR measures distance by firing laser pulses and timing how long they take to bounce back. The basic physics are: 

*distance = speed of light × time / 2*

And the engineering that makes it practical has branched into several quite different technologies, each with distinct trade-offs.

- Direct Time-of-Flight (dToF) sends a short laser pulse and measures the exact time until the reflection arrives. The D500 that we are using uses this. It works at long range (up to 12 m indoors, hundreds of metres), and accuracy doesn't degrade with distance.

- Indirect Time-of-Flight (iToF) continuously modulates the laser and measures the phase shift of the returning wave. Most depth cameras use this, such as Intel RealSense.

## Getting started

### Hardware

- [Arduino UNO Q board](https://store.arduino.cc/pages/uno-q)
- [LDROBOT D500 LiDAR sensor connected via USB](https://www.waveshare.com/wiki/D500_LiDAR_Kit)

### Software

- [Edge Impulse Studio](https://edgeimp.com/ei-arduino)

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

Run the Python server directly:

```
source .venv/bin/activate
python3 python/main.py
```

Or use the Arduino App CLI:

```
arduino-app-cli app start
```

### Open the Web UI

Navigate to `http://<board-ip>:5001` in your browser.


## Edge Impulse Studio integration

### Capture training data

1. Open the **Training** tab in the web UI
2. Enter your Edge Impulse project **API key** (found in Edge Impulse Studio `Dashboard` and then click the `Keys` tab)
3. Set a **label** (e.g., `room`, `corridor`, `wall`)
4. Choose a recording duration and click **Record**
5. Scans are uploaded to your Edge Impulse project as time-series data with 360 features (one distance per degree)

<img width="800" alt="Training the ML model for the space detection with the LiDAR on Arduino UNO Q" src="https://github.com/user-attachments/assets/445a1b27-5c0f-402b-94c1-50f225748287" />


### Running model inference

1. Train a model in Edge Impulse Studio using the captured LiDAR data
2. Deploy as **Arduino UNO Q**
3. Install the runtime: `pip install edge_impulse_linux`
4. Open the **Inference** tab, enter the model path, and click **Start Inference**
5. Classification results appear as live confidence bars overlaid on the map



## Disclaimer

This project is intended for educational and experimental purposes only. It is not hardened for production use. Use responsibly. Do not deploy in any safety-critical environments without proper security, testing, and validation. Try it and improve it!


