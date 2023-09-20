#include "omni_chassis/omni_kinematics.hpp"

motor_interface::msg::MotorGoal OmniKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goals;
    float rot = msg->omega * RADIUS;

    clear_goals(motor_goals);
    add_goal(motor_goals, F, msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw) + rot, 0);
    add_goal(motor_goals, L, - msg->vel_x * sin(chassis_yaw) - msg->vel_y * cos(chassis_yaw) + rot, 0);
    add_goal(motor_goals, B, - msg->vel_x * cos(chassis_yaw) + msg->vel_y * sin(chassis_yaw) + rot, 0);
    add_goal(motor_goals, R, msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw) + rot, 0);

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