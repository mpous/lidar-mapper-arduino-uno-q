/**
 * LiDAR Mapper - MCU sketch for Arduino UNO Q
 *
 * The LDROBOT D500 is connected via USB directly to the Linux MPU.
 * All LiDAR parsing is handled in Python (main.py) using pyserial.
 * This sketch only initialises the Bridge for potential future MCU use.
 */

#include <Arduino_RouterBridge.h>

void setup() {
    Serial.begin(115200);
    Bridge.begin();
    Serial.println("LiDAR Mapper MCU ready");
}

void loop() {
    delay(1000);
}
