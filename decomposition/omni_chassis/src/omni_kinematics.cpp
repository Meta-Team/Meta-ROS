#include "omni_chassis/omni_kinematics.hpp"

motor_interface::msg::MotorGoal OmniKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * RADIUS;

    add_goal(motor_goal, F, + msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw) + rot, 0);
    add_goal(motor_goal, L, - msg->vel_x * sin(chassis_yaw) - msg->vel_y * cos(chassis_yaw) + rot, 0);
    add_goal(motor_goal, B, - msg->vel_x * cos(chassis_yaw) + msg->vel_y * sin(chassis_yaw) + rot, 0);
    add_goal(motor_goal, R, + msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw) + rot, 0);

    return motor_goal;
}

motor_interface::msg::MotorGoal OmniKinematics::chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * RADIUS;

    add_goal(motor_goal, F, + msg->vel_x + rot, 0);
    add_goal(motor_goal, L, - msg->vel_y + rot, 0);
    add_goal(motor_goal, B, - msg->vel_x + rot, 0);
    add_goal(motor_goal, R, + msg->vel_y + rot, 0);

    return motor_goal;
}

void OmniKinematics::clear_goal(motor_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void OmniKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goal, const MotorId rid, const float goal_vel, const float goal_pos)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(goal_vel);
    motor_goal.goal_pos.push_back(goal_pos);
}