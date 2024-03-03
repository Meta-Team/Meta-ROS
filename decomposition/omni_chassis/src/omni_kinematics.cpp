#include "omni_chassis/omni_kinematics.hpp"
#include <cmath>
#include <math.h>

float OmniKinematics::cha_r = 0.255;
float OmniKinematics::wheel_r = 0.76;
float OmniKinematics::decel_ratio = 20.0;
float OmniKinematics::n_offset = 0.0;

motor_interface::msg::MotorGoal OmniKinematics::absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, float gimbal, float motor)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r; // m/s
    float dir = gimbal + motor - n_offset; // direction of the movement against chassis in rad
    float vx = msg->vel_x;
    float vy = msg->vel_y;

    add_goal(motor_goal, "F", + vx * sin(dir) + vy * cos(dir) + rot);
    add_goal(motor_goal, "L", - vx * cos(dir) + vy * sin(dir) + rot);
    add_goal(motor_goal, "B", - vx * sin(dir) - vy * cos(dir) + rot);
    add_goal(motor_goal, "R", + vx * cos(dir) - vy * sin(dir) + rot);

    return motor_goal;
}

motor_interface::msg::MotorGoal OmniKinematics::chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r;

    add_goal(motor_goal, "F", + msg->vel_y + rot);
    add_goal(motor_goal, "L", - msg->vel_x + rot);
    add_goal(motor_goal, "B", - msg->vel_y + rot);
    add_goal(motor_goal, "R", + msg->vel_x + rot);

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