#include "motor_feedback/dm_motor_driver.h"

MotorData DmMotorDriver::process_rx()
{
    MotorData present_data;

    float pos_raw = rx_frame.data[1]<<8 | rx_frame.data[2];
    float vel_raw = rx_frame.data[3]<<4 | rx_frame.data[4]>>4;
    float tor_raw = rx_frame.data[5];

    present_data.position = raw2actual(pos_raw, P_MAX, 12);
    present_data.velocity = raw2actual(vel_raw, V_MAX, 12);
    present_data.torque = raw2actual(tor_raw, T_MAX, 12);

    return present_data;
}