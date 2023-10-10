#include "omni_chassis/omni_kinematics.hpp"

motor_interface::msg::DjiGoal OmniKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::DjiGoal motor_goals;
    float rot = msg->omega * RADIUS;

    set_goal(motor_goals, F, msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw) + rot, 0);
    set_goal(motor_goals, L, - msg->vel_x * sin(chassis_yaw) - msg->vel_y * cos(chassis_yaw) + rot, 0);
    set_goal(motor_goals, B, - msg->vel_x * cos(chassis_yaw) + msg->vel_y * sin(chassis_yaw) + rot, 0);
    set_goal(motor_goals, R, msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw) + rot, 0);

    return motor_goals;
}

motor_interface::msg::DjiGoal OmniKinematics::chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg)
{
    motor_interface::msg::DjiGoal motor_goals;
    float rot = msg->omega * RADIUS;

    set_goal(motor_goals, F, msg->vel_x + rot, 0);
    set_goal(motor_goals, L, msg->vel_y + rot, 0);
    set_goal(motor_goals, B, msg->vel_x + rot, 0);
    set_goal(motor_goals, R, msg->vel_y + rot, 0);

    return motor_goals;
}

void OmniKinematics::set_goal(motor_interface::msg::DjiGoal &motor_goals, const WheelId wheel_id, const float goal_vel, const float goal_pos)
{
    motor_goals.goal_vel[wheel_id] = goal_vel;
    motor_goals.goal_pos[wheel_id] = goal_pos;
}