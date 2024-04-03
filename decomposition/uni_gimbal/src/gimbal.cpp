#include "uni_gimbal/gimbal.h"
#include <algorithm>
#include <cmath>

Gimbal::Gimbal(PidParam yaw, PidParam pitch, float comp)
{
    yaw_param = yaw;
    pitch_param = pitch;
    this->ratio = comp;
    calc_thread = std::thread([this](){
        while (true)
        {
            calc_vel();
            std::this_thread::sleep_for(std::chrono::milliseconds(CALC_FREQ));
        }
    });
}

void Gimbal::set_goal(float goal_yaw_pos, float goal_pitch_pos)
{
    this->goal_yaw_pos = goal_yaw_pos;
    this->goal_pitch_pos = goal_pitch_pos;
    curb(goal_pitch_pos, 15.0f / 180.0f * 3.1415926f); // 15 degree
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
    float upper_goal = goal + 2 * M_PI;
    float lower_goal = goal - 2 * M_PI;

    auto closer = [current](const float a, const float b) -> bool {
        return std::abs(a - current) < std::abs(b - current);
    };

    const auto real_goal = std::min({upper_goal, goal, lower_goal}, closer);

    return real_goal - current;
}

void Gimbal::calc_vel()
{
    // yaw
    float prev_yaw_error = yaw_error;
    yaw_error = min_error(goal_yaw_pos, current_yaw_pos);
    yaw_output.p = yaw_param.kp * yaw_error;
    yaw_output.i += yaw_param.ki * yaw_error * DT; curb(yaw_output.i, V_MAX / 2.0f);
    yaw_output.d = yaw_param.kd * (yaw_error - prev_yaw_error) / DT;
    yaw_vel = yaw_output.sum() - ratio * omega;
    curb(yaw_vel, V_MAX);

    // pitch
    float prev_pitch_error = pitch_error;
    pitch_error = min_error(goal_pitch_pos, current_pitch_pos); // not that necessary
    pitch_output.p = pitch_param.kp * pitch_error;
    pitch_output.i += pitch_param.ki * pitch_error * DT; curb(pitch_output.i, V_MAX / 2.0f);
    pitch_output.d = pitch_param.kd * (pitch_error - prev_pitch_error) / DT;
    pitch_vel = pitch_output.sum();
    curb(pitch_vel, V_MAX);
}