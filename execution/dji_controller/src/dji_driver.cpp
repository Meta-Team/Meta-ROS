#include "dji_controller/dji_driver.h"
#include "dji_controller/can_driver.hpp"
#include <cstdint>
#include <linux/can.h>
#include <memory>

auto DjiDriver::can_0 = std::make_shared<CanDriver>(0);
can_frame DjiDriver::tx_frame1;
can_frame DjiDriver::tx_frame2;

// TODO: pid params to be further tuned
DjiDriver::DjiDriver(int motor_id, MotorType motor_type) :
    p2v_prm(0.0, 0.0, 0.0),
    v2c_prm(0.004, 0.00003, 0.1)
{
    this->motor_id = motor_id;
    this->motor_type = motor_type;
    this->current = 0.0;
    this->goal_pos = 0.0;
    this->goal_vel = 0.0;
    this->vel_error = 0.0;
    this->pos_error = 0.0;
    this->p2v_out = PidOutput();
    this->v2c_out = PidOutput();

    tx_frame1.can_id = 0x200;
    tx_frame1.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame1.data[i] = 0x00;
    tx_frame2.can_id = 0x1ff;
    tx_frame2.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame2.data[i] = 0x00;
}

void DjiDriver::set_goal(float goal_pos, float goal_vel)
{
    this->goal_pos = goal_pos;
    this->goal_vel = goal_vel;
}

void DjiDriver::set_p2v_pid(float kp, float ki, float kd)
{
    p2v_prm.kp = kp;
    p2v_prm.ki = ki;
    p2v_prm.kd = kd;
}

void DjiDriver::set_v2c_pid(float kp, float ki, float kd)
{
    v2c_prm.kp = kp;
    v2c_prm.ki = ki;
    v2c_prm.kd = kd;
}

void DjiDriver::vel2current()
{
    float previous_vel_error = vel_error;
    vel_error = goal_vel - present_data.velocity;

    v2c_out.p = v2c_prm.kp * vel_error;
    v2c_out.i += v2c_prm.ki * vel_error * CONTROL_R;
    v2c_out.d = v2c_prm.kd * (vel_error - previous_vel_error) / CONTROL_R;

    current = v2c_out.sum();

    if (current > 20) current = 20;
    else if (current < -20) current = -20;
}

void DjiDriver::pos2velocity()
{
    float previous_pos_error = pos_error;
    pos_error = goal_pos - present_data.position;

    p2v_out.p = p2v_prm.kp * pos_error;
    p2v_out.i += p2v_prm.ki * pos_error * CONTROL_R;
    p2v_out.d = p2v_prm.kd * (pos_error - previous_pos_error) / CONTROL_R;

    goal_vel = p2v_out.sum();

    if (goal_vel > 20) goal_vel = 20;
    else if (goal_vel < -20) goal_vel = -20;
}

void DjiDriver::write_frame()
{
    if (goal_pos != 0.0) pos2velocity();
    vel2current();
    int16_t current_data = static_cast<int16_t>(current / 20 * 16384);

    if (motor_id <= 4)
    {
        tx_frame1.data[2 * motor_id - 2] = current_data >> 8;
        tx_frame1.data[2 * motor_id - 1] = current_data & 0xff;
    } else {
        tx_frame2.data[2 * (motor_id - 4) - 2] = current_data >> 8;
        tx_frame2.data[2 * (motor_id - 4) - 1] = current_data & 0xff;
    }
}

void DjiDriver::send_frame()
{
    can_0->send_frame(tx_frame1);
    can_0->send_frame(tx_frame2);
}