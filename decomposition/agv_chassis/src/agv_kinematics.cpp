#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>
#include <math.h>

std::unordered_map<std::string, float> AgvKinematics::offsets =
{
    {"LF_D", 0.0},
    {"RF_D", 0.0},
    {"LB_D", 0.0},
    {"RB_D", 0.0}
};

motor_interface::msg::MotorGoal AgvKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);
    float trans_n = msg->vel_tau * sin(yaw_diff) + msg->vel_n * cos(yaw_diff);
    float trans_tao = msg->vel_tau * cos(yaw_diff) - msg->vel_n * sin(yaw_diff);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    add_goal(motor_goal, "LF_D", 0, atan((trans_n + rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RF_D", 0, atan((trans_n + rot_vel) / (trans_tao + rot_vel)));
    add_goal(motor_goal, "LB_D", 0, atan((trans_n - rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RB_D", 0, atan((trans_n - rot_vel) / (trans_tao + rot_vel)));

    add_goal(motor_goal, "LF_V", rss(trans_n + rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RF_V", rss(trans_n + rot_vel, trans_tao + rot_vel), 0);
    add_goal(motor_goal, "LB_V", rss(trans_n - rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RB_V", rss(trans_n - rot_vel, trans_tao + rot_vel), 0);

    return motor_goal;
}

motor_interface::msg::MotorGoal AgvKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);

    float trans_n = msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw);
    float trans_tao = msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    add_goal(motor_goal, "LF_D", 0, atan((trans_n + rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RF_D", 0, atan((trans_n + rot_vel) / (trans_tao + rot_vel)));
    add_goal(motor_goal, "LB_D", 0, atan((trans_n - rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RB_D", 0, atan((trans_n - rot_vel) / (trans_tao + rot_vel)));

    add_goal(motor_goal, "LF_V", rss(trans_n + rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RF_V", rss(trans_n + rot_vel, trans_tao + rot_vel), 0);
    add_goal(motor_goal, "LB_V", rss(trans_n - rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RB_V", rss(trans_n - rot_vel, trans_tao + rot_vel), 0);
    
    return motor_goal;
}

motor_interface::msg::MotorGoal AgvKinematics::chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg)
{
    motor_interface::msg::MotorGoal motor_goal;
    clear_goal(motor_goal);

    float trans_n = msg->vel_y;
    float trans_tao = msg->vel_x;
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    add_goal(motor_goal, "LF_D", 0, atan((trans_n + rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RF_D", 0, atan((trans_n + rot_vel) / (trans_tao + rot_vel)));
    add_goal(motor_goal, "LB_D", 0, atan((trans_n - rot_vel) / (trans_tao - rot_vel)));
    add_goal(motor_goal, "RB_D", 0, atan((trans_n - rot_vel) / (trans_tao + rot_vel)));

    add_goal(motor_goal, "LF_V", rss(trans_n + rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RF_V", rss(trans_n + rot_vel, trans_tao + rot_vel), 0);
    add_goal(motor_goal, "LB_V", rss(trans_n - rot_vel, trans_tao - rot_vel), 0);
    add_goal(motor_goal, "RB_V", rss(trans_n - rot_vel, trans_tao + rot_vel), 0);
    
    return motor_goal;
}

void AgvKinematics::clear_goal(motor_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void AgvKinematics::add_goal(motor_interface::msg::MotorGoal &motor_goal, const std::string& rid, const float goal_vel, const float goal_pos)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(goal_vel);
    motor_goal.goal_pos.push_back(goal_pos + offsets[rid]);
}

float AgvKinematics::rss(float x, float y)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}