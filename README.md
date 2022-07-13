# BEATS BATON

## Object
https://www.multicraftplastics.com/product/TAWH38D

## Code and branches

### main
The Main branch uses [Arduino Nano 33 BLE Sense](https://store-usa.arduino.cc/products/arduino-nano-33-ble-sense) board with built-in 9DOF sensors. The code was used at MUTEK and is the most feature packed version to date.

##### Features
- Bidirectional communication via BLE.
- 9DOF IMU readings output on BLE characteristic "2FFF" in read or notify mode.
- LED light mode on BLE characteristic "3FFF" in write, read or modifiy mode. At the moment, only mode 0 (off) and 1 (on) mode is supported. Adafruit boiler plate patterns are included in the code but not implemented yet.
- Multi-threading parralelize IMU and LED operation and avoid delays and or blocking methods.

Notes: It is at the moment the only code that allow a complete show experience.

##### main-evan
 This is a copy of the Main branch with the addition of the Roll, Pitch and Yaw calculation sent by bluetooth. The sensor fusion is handle by the Madgwick Filter library. This addition to the code has been made by Evan and is used as a checkpoint for futur investigations.

##### main-evan_rpy
This a a copy of main-evan with change that it send only the Raoll, Pitch and Yaw by bluetooth instead of the sensors values. It's been used to benchmark bluetooth transmission performance and kept as it was the only way to receive while keeping a steady signal.

#### Issues of these branches

##### Bluetooth
Bluetooth has constant issues with handling the data transmision and keeping a steady connection.
It seems to come from the limited bandwith of the communication protocol. We can notice more frequent disconnection and lagging data stream when we add additionnal IMU sensors reading in the data sent. We struggle to have any stream of data when we start send Accel + Gyro + Mag + Roll, Pitch and Yaw in the same message.

##### 9DOF
Altough the Gyro, Accel, and Mag sensors seems to give resonably correct values the sensor fusion from Madgwick Filter seems to be faulty. We need to physically turn the boaard around 4 time to get a full 360 degree change in the Roll, Pitch or Yaw values. The sensors also keep drifting overtime.


### esp-32

This is the latest branch and has been developped during December's 2021 Eastern Bloc residency. It serves as performance comparative to the Arduino 33 BLE Sense and will be kept as the starting point to migrate the platform.

It uses a [TinyPICO ESP32](https://www.tinypico.com/) board and a [BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor) 9DOF IMU. 

We chose the TinyPICO for it's small size and it's built-in WIFI chip and the BNO055 for it's self-calibration and inboard sensor fusion capacities.
So far, this combination has proved more robust, seems to provide accurate IMU and orientation data and appear to us has a good direction to pursue.

Note that you can test the code on other ESP32 dev boards other than TinyPICO.

##### Features
- Bidirectionnal communication via MQTT
- 9DOF readings sent on a specific sensor MQTT topic. ie: beats-baton/Accel/x for accelerometer X sensor value.
- Orientation sent on MQTT with the same methodology as the 9DOF data
- OTA - Over the air update for easier maintenance.
- Multi-threading implementation (WIP: Boiler plate code for testing purposes in LED thread)
- Basic LED lights support (ON-OFF) on MQTT topic beats-baton/led


##### To do
- Implement new LED lights pattern and easing functions
- Benchmark MQTT vs OSC for sensors signal output

##### LED patterns to add
- Flash
- Glow
- Wipe
- Noise
- POUM-POUM
- ON-OFF with easing

#### Issues
We still see some slight lags in the data transmission. It seems intermittant and might be caused by the wiring or the speed of the sensors vs the board. Since it was a quick and dirty test to verify the relevance of a platform migration, there's a lot of unnecessary boiler plate code that could be pruned in the upcomming developpement.


### PlatformIO
The Visual Studio Code extention [PlatformIO](https://platformio.org/) is used to make it easier to work with collaborator and share the project dependancies and multiple boards experimentation. Although the content of main.cpp can be copied directly to a .ino script, we strongly recommend using PlatformIO.