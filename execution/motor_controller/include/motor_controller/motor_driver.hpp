#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#define VEL_MODE 0
#define POS_MODE 1
#define MIT_MODE 1

// TODO: add other motor params

class MotorDriver
{
public:
    int motor_id;
    int motor_type;
    can_frame tx_frame;
    static CanDriver* can_0;

    virtual void set_mode() = 0;
    virtual void turn_on() = 0;
    virtual void set_velocity(float goal_vel) = 0;
    virtual void set_position(float goal_pos) = 0;
    virtual void turn_off() = 0;

    virtual ~MotorDriver() = default;

    static float uint_to_float(int x_int, float x_min, float x_max, int bits){
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

    static int float_to_uint(float x, float x_min, float x_max, int bits){
        /// Converts a float to an unsigned int, given range and number of bits
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
};

#endif // MOTOR_DRIVER_HPP