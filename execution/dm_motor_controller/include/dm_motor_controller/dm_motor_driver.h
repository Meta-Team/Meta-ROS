#ifndef DM_MOTOR_DRIVER_H
#define DM_MOTOR_DRIVER_H

#include "dm_motor_config.h"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

class DmMotorDriver
{
public:

    static motor_mode motors_mode[DmMotorCFG::MotorName::MOTOR_COUNT];

    static CanDriver *can_0;

    static can_frame tx_frame[DmMotorCFG::MotorName::MOTOR_COUNT];

    // static constexpr uint64_t start_cmd = 0xfcffffffffffffff;
    static constexpr uint8_t start_cmd[8] = {0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    // static constexpr uint64_t stop_cmd = 0xfdffffffffffffff;
    static constexpr uint8_t stop_cmd[8] = {0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    // static constexpr uint64_t save_zero_cmd = 0xfeffffffffffffff;
    static constexpr uint8_t save_zero_cmd[8] = {0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    // static constexpr uint64_t clear_error_cmd = 0xffffffffffffffff;
    static constexpr uint8_t clear_error_cmd[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

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

    static void init(CanDriver *can);

    static void start(DmMotorCFG::MotorName motorProfile);

    static void stop(DmMotorCFG::MotorName motorProfile);

    static void set_mode(DmMotorCFG::MotorName motorProfile, motor_mode mode);

    static void set_velocity(DmMotorCFG::MotorName motorProfile, float vel);

    static void set_position(DmMotorCFG::MotorName motorProfile, float pos);

    static void set_torque(DmMotorCFG::MotorName motorProfile, float torque);

    static void set_param_mit(DmMotorCFG::MotorName motorProfile, float kp, float kd);

};

#endif // DM_MOTOR_DRIVER_H