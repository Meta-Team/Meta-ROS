#include "motor_feedback/dji_motor_driver.h"
#include "motor_feedback/motor_data.hpp"
#include "motor_feedback/motor_driver.hpp"
#include <string>

DjiMotorDriver::DjiMotorDriver(std::string rid, int hid, MotorType motor_type)
{
    this->rid = rid;
    this->hid = hid;
    this->motor_type = motor_type;
}

void DjiMotorDriver::process_rx()
{
    int16_t pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    int16_t vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    int16_t tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    if (motor_type == M3508)
    {
        if ((int)rx_frame.can_id != 0x200 + hid) return;
        // update only when the frame is for this motor
        present_data.update_pos((float)pos_raw * ENCODER_ANGLE_RATIO);
        present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
        present_data.torque = (float)tor_raw * 16384 / 20; // actually current, Ampere

    } else if (motor_type == M6020)
    {
        if ((int)rx_frame.can_id != 0x204 + hid) return;
        // update only when the frame is for this motor
        present_data.update_pos((float)pos_raw * ENCODER_ANGLE_RATIO);
        present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
        present_data.torque = (float)tor_raw * 16384 / 20; // actually current, Ampere
    }
}