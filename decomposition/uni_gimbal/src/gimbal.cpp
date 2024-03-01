#include "uni_gimbal/gimbal.h"
#include <utility>

Gimbal::Gimbal(PidParam yaw, PidParam pitch)
{
    auto zeros = std::deque<float>(Q_SIZE, 0.0);
    yaw_errors = std::queue<float>(zeros);
    pitch_errors = std::queue<float>(zeros);

    yaw_param = yaw;
    pitch_param = pitch;
}

void Gimbal::set_goal(float goal_yaw_pos, float goal_pitch_pos)
{
    this->goal_yaw_pos = goal_yaw_pos;
    this->goal_pitch_pos = goal_pitch_pos;
}

void Gimbal::update_omega(float omega)
{
    this->omega = omega;
}

void Gimbal::get_feedback(float current_yaw_pos, float current_pitch_pos)
{
    this->current_pitch_pos = current_pitch_pos;
    this->current_yaw_pos = current_yaw_pos;
}

std::pair<float, float> Gimbal::calc_vel()
{
    // yaw
    float prev_yaw_error = yaw_errors.back();
    // pop out the oldest error
    float out_yaw_error = yaw_errors.front();
    yaw_errors.pop();
    // push in the newest error
    float in_yaw_error = goal_yaw_pos - current_yaw_pos;
    yaw_errors.push(in_yaw_error);

    float yaw_p = yaw_param.kp * in_yaw_error;
    float yaw_i = yaw_param.ki * (in_yaw_error - out_yaw_error) * CONTROL_R; // the sum of the queue
    float yaw_d = yaw_param.kd * (in_yaw_error - prev_yaw_error) / CONTROL_R;
    
    yaw_vel = yaw_p + yaw_i + yaw_d - omega; // compensate for the chassis angular velocity
    if (yaw_vel > V_MAX) yaw_vel = V_MAX;
    else if (yaw_vel < -V_MAX) yaw_vel = -V_MAX;

    // pitch
    float prev_pitch_error = pitch_errors.back();
    // pop out the oldest error
    float out_pitch_error = pitch_errors.front();
    pitch_errors.pop();
    // push in the newest error
    float in_pitch_error = goal_pitch_pos - current_pitch_pos;
    pitch_errors.push(in_pitch_error);

    float pitch_p = pitch_param.kp * in_pitch_error;
    float pitch_i = pitch_param.ki * (in_pitch_error - out_pitch_error) * CONTROL_R; // the sum of the queue
    float pitch_d = pitch_param.kd * (in_pitch_error - prev_pitch_error) / CONTROL_R;

    pitch_vel = pitch_p + pitch_i + pitch_d;
    if (pitch_vel > V_MAX) pitch_vel = V_MAX;
    else if (pitch_vel < -V_MAX) pitch_vel = -V_MAX;

    return std::make_pair(yaw_vel, pitch_vel);
}