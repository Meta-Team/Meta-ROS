#include "uni_gimbal/gimbal.h"
#include <algorithm>
#include <cmath>

#if IMU_FB == false
Gimbal::Gimbal(PidParam yaw_p2v, PidParam pitch_p2v, float comp)
{
    yaw_p2v_param = yaw_p2v;
    pitch_p2v_param = pitch_p2v;
    this->ratio = comp;
    calc_thread = std::thread([this](){
        while (true)
        {
            calc_vel();
            std::this_thread::sleep_for(std::chrono::milliseconds(CALC_FREQ));
        }
    });
}
#else // IMU_FB == true
Gimbal::Gimbal(PidParam yaw_p2v, PidParam pitch_p2v, PidParam yaw_v2v, PidParam pitch_v2v, float comp)
{
    yaw_p2v_param = yaw_p2v;
    pitch_p2v_param = pitch_p2v;
    yaw_v2v_param = yaw_v2v;
    pitch_v2v_param = pitch_v2v;
    this->ratio = comp;
    calc_thread = std::thread([this](){
        while (true)
        {
            calc_vel(); // this would overwrite the goal velocity
            calc_vol();
            std::this_thread::sleep_for(std::chrono::milliseconds(CALC_FREQ));
        }
    });
}
#endif // IMU_FB

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

void Gimbal::update_pos_feedback(float yaw_pos, float pitch_pos)
{
    this->current_pitch_pos = pitch_pos;
    this->current_yaw_pos = yaw_pos;
}

#if IMU_FB == true
void Gimbal::update_vel_feedback(float yaw_vel, float pitch_vel)
{
    this->current_pitch_vel = pitch_vel;
    this->current_yaw_vel = yaw_vel;
}
#endif // IMU_FB

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
    float prev_yaw_pos_error = yaw_pos_error;
    yaw_pos_error = min_error(goal_yaw_pos, current_yaw_pos);
    yaw_p2v_output.p = yaw_p2v_param.kp * yaw_pos_error;
    yaw_p2v_output.i += yaw_p2v_param.ki * yaw_pos_error * DT; curb(yaw_p2v_output.i, V_MAX / 2.0f);
    yaw_p2v_output.d = yaw_p2v_param.kd * (yaw_pos_error - prev_yaw_pos_error) / DT;
    goal_yaw_vel = yaw_p2v_output.sum() - ratio * omega;
    curb(goal_yaw_vel, V_MAX);

    // pitch
    float prev_pitch_pos_error = pitch_pos_error;
    pitch_pos_error = min_error(goal_pitch_pos, current_pitch_pos); // not that necessary
    pitch_p2v_output.p = pitch_p2v_param.kp * pitch_pos_error;
    pitch_p2v_output.i += pitch_p2v_param.ki * pitch_pos_error * DT; curb(pitch_p2v_output.i, V_MAX / 2.0f);
    pitch_p2v_output.d = pitch_p2v_param.kd * (pitch_pos_error - prev_pitch_pos_error) / DT;
    goal_pitch_vel = pitch_p2v_output.sum();
    curb(goal_pitch_vel, V_MAX);
}

#if IMU_FB == true
void Gimbal::calc_vol()
{
    // yaw
    float prev_yaw_vel_error = yaw_vel_error;
    yaw_vel_error = goal_yaw_vel - current_yaw_vel;
    yaw_v2v_output.p = yaw_v2v_param.kp * yaw_vel_error;
    yaw_v2v_output.i += yaw_v2v_param.ki * yaw_vel_error * DT; curb(yaw_v2v_output.i, V_MAX / 2.0f);
    yaw_v2v_output.d = yaw_v2v_param.kd * (yaw_vel_error - prev_yaw_vel_error) / DT;
    goal_yaw_vol = yaw_v2v_output.sum();
    curb(goal_yaw_vol, V_MAX);

    // pitch
    float prev_pitch_vel_error = pitch_vel_error;
    pitch_vel_error = goal_pitch_vel - current_pitch_vel;
    pitch_v2v_output.p = pitch_v2v_param.kp * pitch_vel_error;
    pitch_v2v_output.i += pitch_v2v_param.ki * pitch_vel_error * DT; curb(pitch_v2v_output.i, V_MAX / 2.0f);
    pitch_v2v_output.d = pitch_v2v_param.kd * (pitch_vel_error - prev_pitch_vel_error) / DT;
    goal_pitch_vol = pitch_v2v_output.sum();
    curb(goal_pitch_vol, V_MAX);
}
#endif // IMU_FB