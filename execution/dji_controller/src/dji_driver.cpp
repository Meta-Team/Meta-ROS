#include "dji_controller/dji_driver.h"
#include "dji_controller/can_driver.hpp"
#include <cstdint>
#include <linux/can.h>
#include <memory>

std::unique_ptr<CanDriver> DjiDriver::can_0 = std::make_unique<CanDriver>(0);
can_frame DjiDriver::tx_frame_200;
can_frame DjiDriver::tx_frame_1ff;
can_frame DjiDriver::tx_frame_2ff;

can_frame DjiDriver::rx_frame;

// TODO: pid params to be further tuned
DjiDriver::DjiDriver(int motor_id, MotorType motor_type) :
    p2v_prm(0.1, 0.01, 0.1),
    v2c_prm(0.004, 0.00003, 0.1)
{
    this->motor_id = motor_id;
    this->motor_type = motor_type;
    this->p2v_out = PidOutput();
    this->v2c_out = PidOutput();
    init_frame();
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

    this->current = v2c_out.sum();

    // restrict the current
    if (current > I_MAX) this->current = I_MAX;
    else if (current < -I_MAX) this->current = -I_MAX;
}

void DjiDriver::pos2velocity()
{
    float previous_pos_error = pos_error;
    pos_error = goal_pos - present_data.position;

    p2v_out.p = p2v_prm.kp * pos_error;
    p2v_out.i += p2v_prm.ki * pos_error * CONTROL_R;
    p2v_out.d = p2v_prm.kd * (pos_error - previous_pos_error) / CONTROL_R;

    goal_vel = p2v_out.sum();

    // restrict the velocity
    if (goal_vel > V_MAX) this->goal_vel = V_MAX;
    else if (goal_vel < -V_MAX) this->goal_vel = -V_MAX;
}

void DjiDriver::init_frame()
{
    tx_frame_200.can_id = 0x200;
    tx_frame_200.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame_200.data[i] = 0x00;
    
    tx_frame_1ff.can_id = 0x1ff;
    tx_frame_1ff.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame_1ff.data[i] = 0x00;

    tx_frame_2ff.can_id = 0x2ff;
    tx_frame_2ff.can_dlc = 8;
    for (int i = 0; i < 8; i++) tx_frame_2ff.data[i] = 0x00;
}

void DjiDriver::process_rx()
{
    int16_t pos_raw = rx_frame.data[0]<<8 | rx_frame.data[1];
    int16_t vel_raw = rx_frame.data[2]<<8 | rx_frame.data[3];
    int16_t tor_raw = rx_frame.data[4]<<8 | rx_frame.data[5];

    if (motor_type == M3508)
    {
        if ((int)rx_frame.can_id != 0x200 + motor_id) return;
        // update only when the frame is for this motor
        present_data.update_pos((float)pos_raw * ENCODER_ANGLE_RATIO);
        present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
        present_data.torque = (float)tor_raw * 16384 / 20; // actually current, Ampere

    } else if (motor_type == M6020)
    {
        if ((int)rx_frame.can_id != 0x204 + motor_id) return;
        // update only when the frame is for this motor
        present_data.update_pos((float)pos_raw * ENCODER_ANGLE_RATIO);
        present_data.velocity = (float)vel_raw * 3.1415926f / 30.0f; // rpm to rad/s, 2*pi/60
        present_data.torque = (float)tor_raw * 16384 / 20; // actually current, Ampere
    }
}

void DjiDriver::write_tx()
{
    if (goal_pos != 0.0 && goal_vel == 0.0) pos2velocity();
    vel2current();
    int16_t current_data = static_cast<int16_t>(current / I_MAX * 16384);

    if (motor_type == M3508)
    {
        if (motor_id <= 4)
        {
            tx_frame_200.data[2 * motor_id - 2] = (uint8_t)current_data >> 8;
            tx_frame_200.data[2 * motor_id - 1] = (uint8_t)current_data & 0xff;
        } else {
            tx_frame_1ff.data[2 * (motor_id - 4) - 2] = (uint8_t)current_data >> 8;
            tx_frame_1ff.data[2 * (motor_id - 4) - 1] = (uint8_t)current_data & 0xff;
        }
    }
    else if (motor_type == M6020)
    {
        if (motor_id <= 4)
        {
            tx_frame_1ff.data[2 * motor_id - 2] = (uint8_t)current_data >> 8;
            tx_frame_1ff.data[2 * motor_id - 1] = (uint8_t)current_data & 0xff;
        } else {
            tx_frame_2ff.data[2 * (motor_id - 4) - 2] = (uint8_t)current_data >> 8;
            tx_frame_2ff.data[2 * (motor_id - 4) - 1] = (uint8_t)current_data & 0xff;
        }
    }

    
}

void DjiDriver::send_frame()
{
    can_0->send_frame(tx_frame_200);
    can_0->send_frame(tx_frame_1ff);
    can_0->send_frame(tx_frame_2ff);
}

void DjiDriver::get_frame()
{
    can_0->get_frame(rx_frame);
}