#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>

double AgvKinematics::wheel_r = 0.05;
double AgvKinematics::cha_r = 0.3;
double AgvKinematics::decel_ratio = 20.0;
double AgvKinematics::n_offset = 0.0;
double AgvKinematics::yaw_offset = 0.0;

unordered_map<string, double> AgvKinematics::vel = 
{
    // {rid, vel}
    {"LF_V", 0.0},
    {"RF_V", 0.0},
    {"LB_V", 0.0},
    {"RB_V", 0.0},
}; // m/s

unordered_map<string, double> AgvKinematics::pos = 
{
    // {rid, pos}
    {"LF_D", 0.0},
    {"RF_D", 0.0},
    {"LB_D", 0.0},
    {"RB_D", 0.0},
}; // rad

unordered_map<string, double> AgvKinematics::offsets =
{
    {"LF_D", 0.0},
    {"RF_D", 0.0},
    {"LB_D", 0.0},
    {"RB_D", 0.0},
}; // rad

MotorGoal AgvKinematics::natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double yaw_diff)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    double trans_x = msg->vel_x * cos(yaw_diff) - msg->vel_y * sin(yaw_diff);
    double trans_y = msg->vel_x * sin(yaw_diff) + msg->vel_y * cos(yaw_diff);
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);

    return motor_goal;
}

MotorGoal AgvKinematics::absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, double gimbal, double motor)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    double dir = gimbal + (motor - yaw_offset) - n_offset;
    double trans_x = msg->vel_x * cos(dir) - msg->vel_y * sin(dir);
    double trans_y = msg->vel_x * sin(dir) + msg->vel_y * cos(dir);
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);
    
    return motor_goal;
}

MotorGoal AgvKinematics::chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    double trans_x = msg->vel_x;
    double trans_y = msg->vel_y;
    double rot_vel = msg->omega * cha_r / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_x - rot_vel, trans_y + rot_vel);
    add_group_goal(motor_goal, "LB", trans_x - rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RB", trans_x + rot_vel, trans_y - rot_vel);
    add_group_goal(motor_goal, "RF", trans_x + rot_vel, trans_y + rot_vel);
    
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
    else if (vx >= 0)
    {
        pos[dir_id] = atan2(vy, vx);
        vel[vel_id] = velocity;
    }
    else // vx < 0
    {
        pos[dir_id] = atan2(-vy, -vx);
        vel[vel_id] = -velocity;
    }
    // pos is kept between -pi/2 and pi/2

    add_vel_goal(motor_goal, vel_id, vel[vel_id]); // MY_TODO: check this
    add_pos_goal(motor_goal, dir_id, pos[dir_id]);
}

double AgvKinematics::rss(double x, double y)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

bool AgvKinematics::is_zero(double x)
{
    return std::abs(x) < TRIGGER;
}