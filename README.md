# BT_Car - PS3 Controller Controlled Mobile Robot

## Project Overview
BT_Car is an ESP32-based mobile robot that can be controlled through Bluetooth using a PS3 controller. Once powered on, the robot automatically connects to the PS3 controller when it’s also turned on. With this setup, the robot can navigate freely, maintain a specified distance from obstacles using PID control, and perform basic line-following with IR sensors.

## Components
- **ESP32**: DOIT Devkit v1
- **Motor Driver**: L298N Mini Driver
- **Motors**: Two DC TT gear motors with wheels
- **Sensors**:
  - Ultrasonic: HC-SR04 for front obstacle detection
  - Infrared (IR): Two sensors for line-following
- **Indicators**:
  - Power LED: Indicates the robot is powered on
  - Connection LED: Indicates successful connection with the PS3 controller
- **Additional Hardware**:
  - Breadboard and components (wires, resistors)
  - Chassis and battery case for assembly
- **Batteries**: Four 1.5V AA alkaline batteries (providing approximately 6V for the ESP32 and motors)

## Current Features
- **Free Control**: Full manual control using the PS3 controller.
- **Obstacle Distance Control**: Maintains a set front distance using a PID controller, adjustable via R1 and R2 buttons.
- **Basic Line Following**: IR sensors detect a black line; the robot follows by adjusting direction based on sensor input.

## Observed Power Issues
During motor use, the current draw from the 6V battery pack can cause the ESP32 to restart, leading to loss of Bluetooth connection. This issue does not occur when the ESP32 is powered separately via USB, suggesting a need for a more stable or higher-voltage power supply in future iterations.

## Future Improvements
- **Enhanced Obstacle Detection**: Integrate a servo to rotate the HC-SR04 sensor for lateral scanning, allowing the robot to check for obstacles on both left and right sides.
- **Floodfill Algorithm**: Implement pathfinding for maze exploration. This requires:
  - Rotation and Distance Estimation: Plan to use an MPU6050 IMU for rotation estimation. Due to drift, adding a magnetometer would improve heading accuracy.
  - Distance Measurement: Investigate using encoders on the wheels for accurate cell-to-cell distance tracking.

## Dependencies
- [**esp32-ps3**](https://github.com/jvpernis/esp32-ps3): Library for PS3 controller Bluetooth communication with the ESP32 by [jvpernis](https://github.com/jvpernis).
- **ESP32 Board Package**: Required for compiling and uploading code to the ESP32.
