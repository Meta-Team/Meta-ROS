#include "wheel_leg_pid/wheel_leg.h"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

#define NaN std::nan("")

WheelLeg::WheelLeg(Param omega_wv_p, Param obli_wv_p, Param bv_obli_p)
{
    omega_wv = std::make_unique<PidAlgorithm>(omega_wv_p);
    obli_wv = std::make_unique<PidAlgorithm>(obli_wv_p);
    bv_obli = std::make_unique<PidAlgorithm>(bv_obli_p);
}

WheelLeg::~WheelLeg()
{
    // stop PID
    omega_wv->stop();
    obli_wv->stop();
    bv_obli->stop();

    std::string msg = "";
    if (!omega_ready) msg += "omega_wv ";
    if (!obli_ready) msg += "obli_wv ";
    if (!bv_ready) msg += "bv_obli ";
    if (!msg.empty()) {
        std::string full_msg = msg + "stoped without start.";
        RCLCPP_INFO(rclcpp::get_logger("wheel_leg"), "%s", full_msg.c_str());
    }
}

void WheelLeg::omega_feedback(double omega)
{
    omega_wv->set_feedback(omega);

    if (!omega_ready)
    {
        omega_wv->start();
        RCLCPP_INFO(rclcpp::get_logger("wheel_leg"), "omega_wv started");
        omega_ready = true;
    }
}

void WheelLeg::bv_feedback(double bv)
{
    bv_obli->set_feedback(bv);

    if (!bv_ready)
    {
        bv_obli->start();
        RCLCPP_INFO(rclcpp::get_logger("wheel_leg"), "bv_obli started");
        bv_ready = true;
    }
}

void WheelLeg::obli_feedback(double obli)
{
    obli_wv->set_feedback(obli);

    if (!obli_ready)
    {
        obli_wv->start();
        RCLCPP_INFO(rclcpp::get_logger("wheel_leg"), "obli_wv started");
        obli_ready = true;
    }
}

void WheelLeg::set_goal(double bv, double omega)
{
    omega_wv->set_target(omega);
    bv_obli->set_target(bv);
}

void WheelLeg::pass_vel_loop()
{
    while (rclcpp::ok())
    {
        obli_wv->set_target(bv_obli->get_output());
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
}

MotorGoal WheelLeg::get_msg() const
{
    MotorGoal msg{};
    set_wheel_vel(msg);
    set_leg_len(msg);
    return msg;
}

void WheelLeg::add_wheel_goal(MotorGoal& msg, const std::string& motor_id, double goal_vel)
{
    msg.motor_id.push_back(motor_id);
    msg.goal_pos.push_back(NaN);
    msg.goal_vel.push_back(goal_vel);
    msg.goal_tor.push_back(NaN);
}

void WheelLeg::set_wheel_vel(MotorGoal& msg) const
{
    double left_vel = obli_wv->get_output() - omega_wv->get_output();
    add_wheel_goal(msg, "W_L", left_vel);
    double right_vel = obli_wv->get_output() + omega_wv->get_output();
    add_wheel_goal(msg, "W_R", right_vel);
}

void WheelLeg::add_joint_goal(MotorGoal& msg, const std::string& motor_id, double goal_pos)
{
    msg.motor_id.push_back(motor_id);
    msg.goal_pos.push_back(goal_pos);
    msg.goal_vel.push_back(NaN);
    msg.goal_tor.push_back(NaN);
}

void WheelLeg::set_leg_len(MotorGoal& msg) const
{
    double len = 0.25; // TODO: implement this function
    add_joint_goal(msg, "L_L", len_to_angle(len));
    add_joint_goal(msg, "L_R", len_to_angle(len));
}

double WheelLeg::len_to_angle(double /*len*/) const
{
    // TODO: implement this function
    return M_PI_4; 
}