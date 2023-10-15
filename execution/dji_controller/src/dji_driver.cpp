#include "dji_controller/dji_driver.h"

void DjiDriver::update_pos(float pos)
{
    present_pos = pos;
    former_pos = present_pos;
}

void DjiDriver::update_vel(float vel)
{
    present_vel = vel;
    former_vel = present_vel;
}

void DjiDriver::set_current(float &current, float goal)
{
    if (goal != 0)
    {
        current = goal;
    }
    else {
        return;
    }
}

void DjiDriver::set_p2v_pid(float kp, float ki, float kd)
{
    p2v_kp = kp;
    p2v_ki = ki;
    p2v_kd = kd;
}

void DjiDriver::set_v2c_pid(float kp, float ki, float kd)
{
    v2c_kp = kp;
    v2c_ki = ki;
    v2c_kd = kd;
}

float DjiDriver::vel2current(float goal_vel)
{
    float current = 0;

    return current;
}

float DjiDriver::pos2current(float goal_pos)
{
    float current = 0;

    return current;
}

float DjiDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int DjiDriver::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}