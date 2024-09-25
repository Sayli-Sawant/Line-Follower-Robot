# Line Follower

## Introduction
The Line Follower navigates along a distinct line using Infrared (IR) sensors.

### Line Follower Robot
The Line Follower detects a line's position using a Cytron LSA08 IR sensor array, where the Tiva C microcontroller adjusts the speed of the motors based on the line's position relative to the center of the sensor array. A control algorithm ensures the bot corrects its trajectory by adjusting the left and right wheel speeds to minimize position error.


## Components Used:
1. **Tiva C Launchpad** (TM4C123GH6PM, Cortex M4)
2. **Cytron LSA08 IR Sensor Array**
3. **Two Geared DC Motors** (12V, 300 RPM)
4. **L298N H-Bridge Motor Driver**
5. **LiPo Battery** (11.1V, 2200 mAh)
6. **Wheels** (8 cm diameter)
7. Breadboard, Jumper Wires

## Working Principle
- **Line Following**: The bot adjusts wheel speeds based on IR sensor input to follow a line. Position error is calculated based on the line's position relative to the sensor array's center, and wheel speeds are adjusted accordingly.

