#include "dji_controller/dji_driver.h"
#include <cstdint>

CanDriver* DjiDriver::can_0 = new CanDriver(0);

void DjiDriver::set_goal(float goal_pos, float goal_vel)
{
    this->goal_pos = goal_pos;
    this->goal_vel = goal_vel;
}

void DjiDriver::update_pos(float pos)
{
    present_pos = pos;
}

void DjiDriver::update_vel(float vel)
{
    present_vel = vel;
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
    float last_vel_error = vel_error;
    vel_error = goal_vel - present_vel;

    proportional = v2c_kp * vel_error;
    integral += v2c_ki * vel_error * DT;
    derivative = v2c_kd * (vel_error - last_vel_error) / DT;

    current += proportional + integral + derivative;

    return current;
}

float DjiDriver::pos2velocity(float goal_pos)
{
    float velocity = 0;

    float last_pos_error = pos_error;
    pos_error = goal_pos - present_pos;

    proportional = p2v_kp * pos_error;
    integral += p2v_ki * pos_error * DT;
    derivative = p2v_kd * (pos_error - last_pos_error) / DT;

    velocity += proportional + integral + derivative;

    return velocity;
}

float DjiDriver::pos2current(float goal_pos)
{
    float velocity = pos2velocity(goal_pos);
    return vel2current(velocity);
}

void DjiDriver::write_frame(can_frame &tx_frame)
{
    if (goal_pos != 0.0) goal_vel = pos2velocity(goal_pos);
    current = vel2current(goal_vel);
    std::uint16_t current_data = DjiDriver::float_to_uint(current, -I_MAX, I_MAX, 16);

    if (motor_id <= 4)
    {
        tx_frame.data[2*motor_id - 2] = current_data >> 8;
        tx_frame.data[2*motor_id - 1] = current_data & 0xff;
        tx_frame.can_id = 0x200;
    } else {
        tx_frame.data[2*(motor_id-4) - 2] = current_data >> 8;
        tx_frame.data[2*(motor_id-4) - 1] = current_data & 0xff;
        tx_frame.can_id = 0x1ff;
    }
}

void DjiDriver::send_frame(can_frame &tx_frame)
{
    can_0->send_frame(tx_frame);
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