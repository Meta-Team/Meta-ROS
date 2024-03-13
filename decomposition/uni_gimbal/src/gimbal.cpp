#include "uni_gimbal/gimbal.h"
#include <algorithm>

Gimbal::Gimbal(PidParam yaw, PidParam pitch)
{
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

void Gimbal::curb(float &val, float max)
{
    if (val > max) val = max;
    else if (val < -max) val = -max;
}

float Gimbal::min_error(float goal, float current)
{
    constexpr static float PI = 3.1415926f;

    float upper_goal = goal + 2 * PI;
    float lower_goal = goal - 2 * PI;

    auto closer = [current](const float a, const float b) -> bool {
        return std::abs(a - current) < std::abs(b - current);
    };

    const auto real_goal = std::min({upper_goal, goal, lower_goal}, closer);

    return real_goal - current;
}

std::pair<float, float> Gimbal::calc_vel()
{
    // yaw
    float prev_yaw_error = yaw_error;
    yaw_error = min_error(goal_yaw_pos, current_yaw_pos);
    yaw_output.p = yaw_param.kp * yaw_error;
    yaw_output.i += yaw_param.ki * yaw_error * DT;
    yaw_output.d = yaw_param.kd * (yaw_error - prev_yaw_error) / DT;
    yaw_vel = yaw_output.sum();
    curb(yaw_vel, V_MAX);

    // pitch
    float prev_pitch_error = pitch_error;
    pitch_error = min_error(goal_pitch_pos, current_pitch_pos); // not that necessary
    pitch_output.p = pitch_param.kp * pitch_error;
    pitch_output.i += pitch_param.ki * pitch_error * DT;
    pitch_output.d = pitch_param.kd * (pitch_error - prev_pitch_error) / DT;
    pitch_vel = pitch_output.sum();
    curb(pitch_vel, V_MAX);

    return std::make_pair(yaw_vel, pitch_vel);
}