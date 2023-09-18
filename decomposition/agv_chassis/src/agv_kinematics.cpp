#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>
#include <math.h>

motor_interface::msg::MotorGoal AgvKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float chassis_yaw, float gimbal_pos)
{
    motor_interface::msg::MotorGoal motor_goals;
    return motor_goals;
}

motor_interface::msg::MotorGoal AgvKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goals;
    clear_goals(motor_goals);
    float rot_sin = msg->omega * RADIUS * sin(PI/4 - chassis_yaw);
    float rot_cos = msg->omega * RADIUS * cos(PI/4 - chassis_yaw);
    add_goal(motor_goals, LF0, 0, atan((msg->vel_x + rot_sin) / (msg->vel_y - rot_cos)));
    add_goal(motor_goals, RF0, 0, atan((msg->vel_x + rot_cos) / (msg->vel_y + rot_sin)));
    add_goal(motor_goals, LB0, 0, atan((msg->vel_x - rot_cos) / (msg->vel_y - rot_sin)));
    add_goal(motor_goals, RB0, 0, atan((msg->vel_x - rot_sin) / (msg->vel_y + rot_cos)));
    add_goal(motor_goals, LF1, rss(msg->vel_x + rot_sin, msg->vel_y - rot_cos), 0);
    add_goal(motor_goals, RF1, rss(msg->vel_x + rot_cos, msg->vel_y + rot_sin), 0);
    add_goal(motor_goals, LB1, rss(msg->vel_x - rot_cos, msg->vel_y - rot_sin), 0);
    add_goal(motor_goals, RB1, rss(msg->vel_x - rot_sin, msg->vel_y + rot_cos), 0);
    return motor_goals;
}

void AgvKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goals, const int motor_id, const float goal_vel, const float goal_pos)
{
    motor_goals.motor_id.push_back(motor_id);
    motor_goals.goal_vel.push_back(goal_vel);
    motor_goals.goal_pos.push_back(goal_pos);
}

void AgvKinematics::clear_goals(motor_interface::msg::MotorGoal &motor_goals)
{
    motor_goals.motor_id.clear();
    motor_goals.goal_pos.clear();
    motor_goals.goal_vel.clear();
}

float AgvKinematics::rss(float x, float y)
{
    return std::sqrt(x * x + y * y);
}