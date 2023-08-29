#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

class MotorDriver
{
public:
    int motor_id;
    int motor_type;
    can_frame rx_frame;
    static CanDriver* can_0;

    virtual float get_velocity() = 0;
    virtual float get_position() = 0;

    virtual ~MotorDriver() = default;
};

#endif // MOTOR_DRIVER_HPP