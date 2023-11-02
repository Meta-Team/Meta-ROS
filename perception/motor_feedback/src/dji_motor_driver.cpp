#include "motor_feedback/dji_motor_driver.h"
#include "motor_feedback/motor_data.hpp"
#include "motor_feedback/motor_driver.hpp"

MotorData DjiMotorDriver::process_rx()
{
    switch (motor_type)
    {
    case M3508:
        return process_rx_3508();
    case M2006:
        return process_rx_2006();
    case M6020:
        return process_rx_6020();
    }
}

MotorData DjiMotorDriver::process_rx_3508()
{
    MotorData present_data;

    float pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    float vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    float tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    present_data.position = raw2actual(pos_raw, POS_MAX, 16);
    present_data.velocity = raw2actual(vel_raw, VEL_MAX, 16);
    present_data.torque = raw2actual(tor_raw, TOR_MAX, 16);

    return present_data;
}

MotorData DjiMotorDriver::process_rx_2006()
{
    MotorData present_data;

    float pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    float vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    float tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    present_data.position = raw2actual(pos_raw, POS_MAX, 16);
    present_data.velocity = raw2actual(vel_raw, VEL_MAX, 16);
    present_data.torque = raw2actual(tor_raw, TOR_MAX, 16);

    return present_data;
}

MotorData DjiMotorDriver::process_rx_6020()
{
    MotorData present_data;

    float pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    float vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    float tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    present_data.position = raw2actual(pos_raw, POS_MAX, 16);
    present_data.velocity = raw2actual(vel_raw, VEL_MAX, 16);
    present_data.torque = raw2actual(tor_raw, TOR_MAX, 16);

    return present_data;
}