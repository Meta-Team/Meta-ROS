#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

MotorGoal AgvKinematics::stop = [] {
    MotorGoal stop_;
    auto set_zero_vel = [&stop_](const string &rid) {
        stop_.motor_id.push_back(rid);
        stop_.goal_tor.push_back(NaN);
        stop_.goal_vel.push_back(0.0);
        stop_.goal_pos.push_back(NaN);
    };
    std::array rid = {"LF_V", "RF_V", "LB_V", "RB_V",
        "LF_D", "RF_D", "LB_D", "RB_D"
    };
    for (const auto &r : rid) set_zero_vel(r);
    return stop_;
}();

AgvKinematics::AgvKinematics(double wheel_r, double cha_r, double decel_ratio, double n_offset, double yaw_offset)
    : wheel_r(wheel_r), cha_r(cha_r), decel_ratio(decel_ratio), n_offset(n_offset), yaw_offset(yaw_offset)
{
    vel = {
        // {rid, vel}
        {"LF_V", 0.0},
        {"RF_V", 0.0},
        {"LB_V", 0.0},
        {"RB_V", 0.0},
    }; // m/s

    pos = {
        // {rid, {round, angle}}
        {"LF_D", {0, 0.0}},
        {"RF_D", {0, 0.0}},
        {"LB_D", {0, 0.0}},
        {"RB_D", {0, 0.0}},
    }; // rad

    offsets = {
        {"LF_D", 0.0},
        {"RF_D", 0.0},
        {"LB_D", 0.0},
        {"RB_D", 0.0},
    }; // rad
}

void AgvKinematics::set_offsets(double lf, double rf, double lb, double rb)
{
    offsets["LF_D"] = lf;
    offsets["RF_D"] = rf;
    offsets["LB_D"] = lb;
    offsets["RB_D"] = rb;
}

void AgvKinematics::natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double yaw_diff)
{
    clear_goal(motor_goal);

    double trans_x = msg->vel_x * cos(yaw_diff) - msg->vel_y * sin(yaw_diff);
    double trans_y = msg->vel_x * sin(yaw_diff) + msg->vel_y * cos(yaw_diff);
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);

    last_rec = rclcpp::Clock().now().seconds();
}

void AgvKinematics::absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, double gimbal, double motor)
{
    clear_goal(motor_goal);

    double dir = gimbal + (motor - yaw_offset) - n_offset;
    double trans_x = msg->vel_x * cos(dir) - msg->vel_y * sin(dir);
    double trans_y = msg->vel_x * sin(dir) + msg->vel_y * cos(dir);
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);

    last_rec = rclcpp::Clock().now().seconds();
}

void AgvKinematics::chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg)
{
    clear_goal(motor_goal);

    double trans_x = msg->vel_x;
    double trans_y = msg->vel_y;
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);

    last_rec = rclcpp::Clock().now().seconds();
}

MotorGoal AgvKinematics::get_motor_goal() const
{
    auto now = rclcpp::Clock().now().seconds();
    if (now - last_rec > 0.3) return stop;
    return motor_goal;
}

void AgvKinematics::clear_goal(MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_tor.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void AgvKinematics::add_vel_goal(MotorGoal &motor_goal, const string& rid, const double goal_vel)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_tor.push_back(NaN);
    motor_goal.goal_vel.push_back(goal_vel / wheel_r * decel_ratio); // convert to rad/s
    motor_goal.goal_pos.push_back(NaN);
}

void AgvKinematics::add_pos_goal(MotorGoal &motor_goal, const string &rid, const double goal_pos)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_tor.push_back(NaN);
    motor_goal.goal_vel.push_back(NaN);
    motor_goal.goal_pos.push_back(DIR * goal_pos + offsets[rid]); // already in rad
}

void AgvKinematics::add_group_goal(MotorGoal &motor_goal, const string& which, double vx, double vy)
{
    string dir_id = which + "_D";
    string vel_id = which + "_V";

    double velocity = rss(vx, vy);
    if (is_zero(velocity)) // if vel is 0, keep the direction
    {
        // pos[dir_id] does not change
        vel[vel_id] = 0;
    }
    else
    {
        const auto& [round, angle, reverse] = closest_angle(std::atan2(vy, vx), pos[dir_id]);
        vel[vel_id] = reverse ? -velocity : velocity;
        pos[dir_id] = {round, angle};
    }

    add_vel_goal(motor_goal, vel_id, vel[vel_id]);
    auto cumu_pos = pos[dir_id].first * 2 * M_PI + pos[dir_id].second;
    add_pos_goal(motor_goal, dir_id, cumu_pos);
}

double AgvKinematics::rss(double x, double y)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

bool AgvKinematics::is_zero(double x)
{
    return std::abs(x) < TRIGGER;
}

double AgvKinematics::min_diff(const double& a, const double& b)
{
    double diff = a - b;
    while (diff > M_PI) diff -= 2 * M_PI;
    while (diff < -M_PI) diff += 2 * M_PI;
    return diff;
}

void AgvKinematics::normalize(pair<int, double>& pos)
{
    auto& [round, angle] = pos;
    while (angle < 0)
    {
        --round;
        angle += 2 * M_PI;
    }
    while (angle >= 2 * M_PI)
    {
        ++round;
        angle -= 2 * M_PI;
    }
}

tuple<int, double, bool> AgvKinematics::closest_angle(const double& new_goal, const pair<int, double>& prev_pos)
{
    auto result = prev_pos;
    const auto& [prev_round, prev_angle] = prev_pos;
    const auto diff = min_diff(prev_angle, new_goal); // prev_angle - new_goal

    if (std::abs(diff) < M_PI / 2) // don't need to reverse
    {
        result.second = prev_angle - diff;
        normalize(result);
        return {result.first, result.second, false};
    }
    else // need to reverse
    {
        auto reverse_goal = new_goal + M_PI;
        if (reverse_goal >= 2 * M_PI) reverse_goal -= 2 * M_PI;
        const auto reverse_diff = min_diff(prev_angle, reverse_goal);
        result.second = prev_angle - reverse_diff;
        normalize(result);
        return {result.first, result.second, true};
    }
}