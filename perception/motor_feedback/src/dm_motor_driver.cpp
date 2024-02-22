#include "motor_feedback/dm_motor_driver.h"

DmMotorDriver::DmMotorDriver(std::string rid, int hid)
{
    this->rid = rid;
    this->hid = hid;
}

float DmMotorDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int DmMotorDriver::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void DmMotorDriver::process_rx()
{
    float pos_raw = rx_frame.data[1]<<8 | rx_frame.data[2];
    float vel_raw = rx_frame.data[3]<<4 | rx_frame.data[4]>>4;
    float tor_raw = rx_frame.data[5];

    present_data.position = uint_to_float(pos_raw, -P_MAX, P_MAX, 16);
    present_data.velocity = uint_to_float(vel_raw, -V_MAX, V_MAX, 12);
    present_data.torque = uint_to_float(tor_raw, -T_MAX, T_MAX, 12);
}