#include "omni_chassis/omni_kinematics.hpp"

float OmniKinematics::cha_r = 0.3;
float OmniKinematics::wheel_r = 0.1;
float OmniKinematics::decel_ratio = 20.0;

motor_interface::msg::MotorGoal OmniKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r;

    add_goal(motor_goal, "F", + msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw) + rot);
    add_goal(motor_goal, "L", - msg->vel_x * sin(chassis_yaw) - msg->vel_y * cos(chassis_yaw) + rot);
    add_goal(motor_goal, "B", - msg->vel_x * cos(chassis_yaw) + msg->vel_y * sin(chassis_yaw) + rot);
    add_goal(motor_goal, "R", + msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw) + rot);

    return motor_goal;
}

motor_interface::msg::MotorGoal OmniKinematics::chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r;

    add_goal(motor_goal, "F", + msg->vel_x + rot);
    add_goal(motor_goal, "L", - msg->vel_y + rot);
    add_goal(motor_goal, "B", - msg->vel_x + rot);
    add_goal(motor_goal, "R", + msg->vel_y + rot);

    return motor_goal;
}

void OmniKinematics::clear_goal(motor_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void OmniKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goal, const std::string& rid, const float goal_vel)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(DIRE * goal_vel / wheel_r * decel_ratio); // convert to rad/s
    motor_goal.goal_pos.push_back(0.0);
}