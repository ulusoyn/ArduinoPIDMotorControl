# ArduinoPIDMotorControl
This repo is created to PID control dc motors. The microcontroller reads the angular velocity commands (in rad/s) and determines desired velocity. Then reads encoder and calculates the actual velocity. MotorController is the bridge between L298N and microcontroller (I used pi pico w).
