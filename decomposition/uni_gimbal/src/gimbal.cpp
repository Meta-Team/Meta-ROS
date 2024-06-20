#include "uni_gimbal/gimbal.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"

MotorGoal Gimbal::stop = [] {
    MotorGoal stop_;
    auto set_zero_vel = [&stop_](const std::string &rid) {
        stop_.motor_id.push_back(rid);
        stop_.goal_tor.push_back(NaN);
        stop_.goal_vel.push_back(0.0);
        stop_.goal_pos.push_back(NaN);
    };
    std::array rids = {"PITCH", "YAW"};
    for (const auto &id : rids) set_zero_vel(id);
    return stop_;
}();

#if IMU_FB == false
Gimbal::Gimbal(PidParam yaw_p2v_param, PidParam pitch_p2v_param)
{
    auto make_pid = [](const PidParam &param, double max_output, double max_i) {
        return std::make_unique<PidAlgorithm>(param.kp, param.ki, param.kd, CALC_FREQ, max_output, max_i);
    };
    yaw_p2v = make_pid(yaw_p2v_param, 2, 1);
    pitch_p2v = make_pid(pitch_p2v_param, 2, 1);

    yaw_p2v->start();
    pitch_p2v->start();
}
#else // IMU_FB == true
Gimbal::Gimbal(PidParam yaw_p2v_param, PidParam pitch_p2v_param, PidParam yaw_v2v_param, PidParam pitch_v2v_param)
{
    auto make_pid = [](const PidParam &param, double max_output, double max_i) {
        return std::make_unique<PidAlgorithm>(param.kp, param.ki, param.kd, CALC_FREQ, max_output, max_i);
    };
    yaw_p2v = make_pid(yaw_p2v_param, 5, 3);
    pitch_p2v = make_pid(pitch_p2v_param, 2, 1);
    yaw_v2v = make_pid(yaw_v2v_param, 20, 10);
    pitch_v2v = make_pid(pitch_v2v_param, 20, 10);

    yaw_p2v->start();
    pitch_p2v->start();
    yaw_v2v->start();
    pitch_v2v->start();

    vel_thread = std::thread(&Gimbal::vel_loop, this);
}
#endif // IMU_FB

void Gimbal::set_goal(double goal_yaw_pos, double goal_pitch_pos)
{
    last_rec = rclcpp::Clock().now().seconds();

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
    last_rec = rclcpp::Clock().now().seconds();

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

MotorGoal Gimbal::get_motor_goal() const
{
    auto now = rclcpp::Clock().now().seconds();
    if (now - last_rec > 0.1) return stop;

    MotorGoal motor_goal{};
#if IMU_FB == false
    motor_goal.motor_id.push_back("YAW");
    motor_goal.goal_tor.push_back(NaN);
    motor_goal.goal_vel.push_back(yaw_p2v->get_output());
    motor_goal.goal_pos.push_back(NaN);

    motor_goal.motor_id.push_back("PITCH");
    motor_goal.goal_tor.push_back(NaN);
    motor_goal.goal_vel.push_back(pitch_p2v->get_output());
    motor_goal.goal_pos.push_back(NaN);
#else // IMU_FB == true
    motor_goal.motor_id.push_back("YAW");
    motor_goal.goal_tor.push_back(yaw_v2v->get_output());
    motor_goal.goal_vel.push_back(NaN);
    motor_goal.goal_pos.push_back(NaN);
    motor_goal.motor_id.push_back("PITCH");
    motor_goal.goal_tor.push_back(pitch_v2v->get_output());
    motor_goal.goal_vel.push_back(NaN);
    motor_goal.goal_pos.push_back(NaN);
#endif // IMU_FB
    return motor_goal;
}

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