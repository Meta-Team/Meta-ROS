#include "omni_chassis/omni_kinematics.hpp"
#include <cmath>
#include <math.h>

MotorGoal OmniKinematics::stop = [] {
    MotorGoal stop_;
    auto set_zero_vel = [&stop_](const std::string &rid) {
        stop_.motor_id.push_back(rid);
        stop_.goal_tor.push_back(NaN);
        stop_.goal_vel.push_back(0.0);
        stop_.goal_pos.push_back(NaN);
    };
    std::array rids = {"L", "F", "B", "R"};
    for (const auto &id : rids) set_zero_vel(id);
    return stop_;
}();

OmniKinematics::OmniKinematics(double wheel_r, double cha_r, double decel_ratio, double n_offset, double yaw_offset)
    : wheel_r(wheel_r), cha_r(cha_r), decel_ratio(decel_ratio), n_offset(n_offset), yaw_offset(yaw_offset)
{
    // nothing
}

void OmniKinematics::absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, float gimbal, float motor)
{
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r; // m/s
    float dir = gimbal + (motor - yaw_offset) - n_offset; // direction of the movement against chassis in rad
    float vx = msg->vel_x;
    float vy = msg->vel_y;

    add_goal(motor_goal, "F", + vx * sin(dir) + vy * cos(dir) + rot);
    add_goal(motor_goal, "L", - vx * cos(dir) + vy * sin(dir) + rot);
    add_goal(motor_goal, "B", - vx * sin(dir) - vy * cos(dir) + rot);
    add_goal(motor_goal, "R", + vx * cos(dir) - vy * sin(dir) + rot);

    last_rec = rclcpp::Clock().now().seconds();
}

void OmniKinematics::chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg)
{
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r;
    float vx = msg->vel_x;
    float vy = msg->vel_y;

    add_goal(motor_goal, "F", + vy + rot);
    add_goal(motor_goal, "L", - vx + rot);
    add_goal(motor_goal, "B", - vy + rot);
    add_goal(motor_goal, "R", + vx + rot);

    last_rec = rclcpp::Clock().now().seconds();
}

void OmniKinematics::natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, float motor)
{
    clear_goal(motor_goal);
    float rot = msg->omega * cha_r;
    float dir = motor - yaw_offset;
    float vx = msg->vel_x;
    float vy = msg->vel_y;

    add_goal(motor_goal, "F", + vx * sin(dir) + vy * cos(dir) + rot);
    add_goal(motor_goal, "L", - vx * cos(dir) + vy * sin(dir) + rot);
    add_goal(motor_goal, "B", - vx * sin(dir) - vy * cos(dir) + rot);
    add_goal(motor_goal, "R", + vx * cos(dir) - vy * sin(dir) + rot);

    last_rec = rclcpp::Clock().now().seconds();
}

MotorGoal OmniKinematics::get_motor_goal() const
{
    auto now = rclcpp::Clock().now().seconds();
    if (now - last_rec > 0.1) return stop;
    return motor_goal;
}

void OmniKinematics::clear_goal(device_interface::msg::MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
    motor_goal.goal_tor.clear();
}

void OmniKinematics::add_goal(device_interface::msg::MotorGoal &motor_goal, const std::string& rid, const float goal_vel)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(DIR * goal_vel / wheel_r * decel_ratio); // convert to rad/s
    motor_goal.goal_pos.push_back(NaN);
    motor_goal.goal_tor.push_back(NaN);
}