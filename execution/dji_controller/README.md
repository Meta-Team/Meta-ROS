# dji_controller

## Description

This package is used to control the DJI motors through CAN bus.

Each can_frame delivers 4 values (id from 1 to 4, from 5 to 8, or from 9 to 12). each value is a 16-bit integer, taking 2 bytes. So, 4 motors should be controlled at a time

The bitrate of the CAN communication should be 1M. CAN frames should be continuously sent to the motors to keep them running. In this case, frames are sent at 100Hz

## Pre-requisites

### Hardware

Use USB-CAN adapter to connect the computer to the DJI motors.

### Software

Enable CAN interface in the computer. We take CAN0 as an example.

```shell
sudo ip link set can0 up type can bitrate 1000000
```

This must be done before runtime and every time the computer is restarted.

## Usage

### Configuration

For DJI motors:

1. Each entry in `motor.ports` should be "CANx" where x is the number of the CAN interface. For example, `"CAN0"`.

2. All entries in `motor.brands` should be `"DJI"`.

3. Each entry in `motor.types` should be one of `"3508"`, `"6020"`, and `"2006"`.

4. Each entry in `motor.hids` should be the hardware ID of the motor. It is between `1` to `8` for 3508 and 2006, `5` to `12` for 2060.

### Runtime

Use `device_interface/msg/MotorGoal` to control the motors.

```
string[] motor_id
float64[] goal_tor # can be current or voltage, A or V
float64[] goal_vel # rad/s
float64[] goal_pos # rad
```

The `motor_id` should be the same as `motor.rids` in the configuration file. Set one of the three goals to a valid value and the other two to `.nan`.