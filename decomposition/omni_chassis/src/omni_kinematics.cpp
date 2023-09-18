#include "omni_chassis/omni_kinematics.hpp"

motor_interface::msg::MotorGoal OmniKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float chassis_yaw, float gimbal_yaw)
{
    motor_interface::msg::MotorGoal motor_goals;

    return motor_goals;
}

motor_interface::msg::MotorGoal OmniKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goals;

    return motor_goals;
}

void OmniKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goals, const int motor_id, const float goal_vel, const float goal_pos)
{
    motor_goals.motor_id.push_back(motor_id);
    motor_goals.goal_vel.push_back(goal_vel);
    motor_goals.goal_pos.push_back(goal_pos);
}

void OmniKinematics::clear_goals(motor_interface::msg::MotorGoal &motor_goals)
{
    motor_goals.motor_id.clear();
    motor_goals.goal_pos.clear();
    motor_goals.goal_vel.clear();
}

float OmniKinematics::rss(float x, float y)
{
    return std::sqrt(x * x + y * y);
}