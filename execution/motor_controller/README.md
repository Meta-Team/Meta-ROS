# motor_controller

This package is responsible for controlling all the motors of the robot.

It supports DJI RoboMaster motors, DM motors(TODO) and Unitree motors.

## Structure

The core of the package is in `src` and `include/motor_controller`.

All other directories(`lib`, `third_party`, and all under `include` except `motor_controller`)
are from the [unitree_actuator_sdk](https://github.com/unitreerobotics/unitree_actuator_sdk) repository.
