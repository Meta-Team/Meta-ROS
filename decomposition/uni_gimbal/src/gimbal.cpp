#include "uni_gimbal/gimbal.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <rclcpp/utilities.hpp>

#if IMU_FB == false
Gimbal::Gimbal(PidParam yaw_p2v_param, PidParam pitch_p2v_param)
{
    yaw_p2v = std::make_unique<PidAlgorithm>(yaw_p2v_param.kp, yaw_p2v_param.ki, yaw_p2v_param.kd, CALC_FREQ);
    pitch_p2v = std::make_unique<PidAlgorithm>(pitch_p2v_param.kp, pitch_p2v_param.ki, pitch_p2v_param.kd, CALC_FREQ);
}
#else // IMU_FB == true
Gimbal::Gimbal(PidParam yaw_p2v_param, PidParam pitch_p2v_param, PidParam yaw_v2v_param, PidParam pitch_v2v_param)
{
    yaw_p2v = std::make_unique<PidAlgorithm>(yaw_p2v_param.kp, yaw_p2v_param.ki, yaw_p2v_param.kd, CALC_FREQ);
    pitch_p2v = std::make_unique<PidAlgorithm>(pitch_p2v_param.kp, pitch_p2v_param.ki, pitch_p2v_param.kd, CALC_FREQ);
    yaw_v2v = std::make_unique<PidAlgorithm>(yaw_v2v_param.kp, yaw_v2v_param.ki, yaw_v2v_param.kd, CALC_FREQ);
    pitch_v2v = std::make_unique<PidAlgorithm>(pitch_v2v_param.kp, pitch_v2v_param.ki, pitch_v2v_param.kd, CALC_FREQ);

    vel_thread = std::thread(&Gimbal::vel_loop, this);
}
#endif // IMU_FB

void Gimbal::set_goal(double goal_yaw_pos, double goal_pitch_pos)
{
    // find the closest cumulated yaw position
    double cur_cumu_yaw_pos = yaw_p2v->get_feedback();
    double goal_cumu_yaw_pos = cur_cumu_yaw_pos + min_error(goal_yaw_pos, cur_cumu_yaw_pos);
    yaw_p2v->set_target(goal_cumu_yaw_pos);

    // limit the pitch angle
    if (goal_pitch_pos > 0.25) goal_pitch_pos = 0.25;
    if (goal_pitch_pos < -0.25) goal_pitch_pos = -0.25;
    pitch_p2v->set_target(goal_pitch_pos);
}

void Gimbal::update_pos_feedback(double fb_yaw_pos, double fb_pitch_pos)
{
    // find the closest cumulated yaw position
    double cur_cumu_yaw_pos = yaw_p2v->get_feedback();
    double fb_cumu_yaw_pos = cur_cumu_yaw_pos + min_error(fb_yaw_pos, cur_cumu_yaw_pos);
    yaw_p2v->set_feedback(fb_cumu_yaw_pos);

    pitch_p2v->set_feedback(fb_pitch_pos);
}

#if IMU_FB == true
void Gimbal::update_vel_feedback(double yaw_vel, double pitch_vel)
{
    yaw_v2v->set_feedback(yaw_vel);
    pitch_v2v->set_feedback(pitch_vel);
}
#endif // IMU_FB

double Gimbal::min_error(double goal, double current)
{
    double diff = goal - current;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

#if IMU_FB == true
void Gimbal::vel_loop()
{
    while (rclcpp::ok())
    {
        double yaw_goal_vel = yaw_p2v->get_output();
        double pitch_goal_vel = pitch_p2v->get_output();
        yaw_v2v->set_target(yaw_goal_vel);
        pitch_v2v->set_target(pitch_goal_vel);
    }
}
#endif // IMU_FB