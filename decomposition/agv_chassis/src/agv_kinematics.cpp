#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>

motor_interface::msg::MotorGoal AgvKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr nat_msg)
{
    motor_interface::msg::MotorGoal motor_goals;
    add_goal(motor_goals, LF0, 0, atan((nat_msg->vel_n + ROT_COMP) / (nat_msg->vel_tau - ROT_COMP)));
    add_goal(motor_goals, RF0, 0, atan((nat_msg->vel_n + ROT_COMP) / (nat_msg->vel_tau + ROT_COMP)));
    add_goal(motor_goals, LB0, 0, atan((nat_msg->vel_n - ROT_COMP) / (nat_msg->vel_tau - ROT_COMP)));
    add_goal(motor_goals, RB0, 0, atan((nat_msg->vel_n - ROT_COMP) / (nat_msg->vel_tau + ROT_COMP)));
    add_goal(motor_goals, LF1, rss(nat_msg->vel_n + ROT_COMP, nat_msg->vel_tau - ROT_COMP), 0);
    add_goal(motor_goals, RF1, rss(nat_msg->vel_n + ROT_COMP, nat_msg->vel_tau + ROT_COMP), 0);
    add_goal(motor_goals, LB1, rss(nat_msg->vel_n - ROT_COMP, nat_msg->vel_tau - ROT_COMP), 0);
    add_goal(motor_goals, RB1, rss(nat_msg->vel_n - ROT_COMP, nat_msg->vel_tau + ROT_COMP), 0);
    return motor_goals;
}

motor_interface::msg::MotorGoal AgvKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg, float current_theta)
{
    motor_interface::msg::MotorGoal motor_goals;
    // TODO: implement absolute_decompo
    return motor_goals;
}

void AgvKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goals, const int motor_id, const float goal_vel, const float goal_pos)
{
    motor_goals.motor_id.push_back(motor_id);
    motor_goals.goal_vel.push_back(goal_vel);
    motor_goals.goal_pos.push_back(goal_pos);
}

float AgvKinematics::rss(float x, float y)
{
    return std::sqrt(x * x + y * y);
}