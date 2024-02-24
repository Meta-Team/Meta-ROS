#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>

unordered_map<string, float> AgvKinematics::vel = 
{
    {"LF_V", 0.0},
    {"RF_V", 0.0},
    {"LB_V", 0.0},
    {"RB_V", 0.0}
};

unordered_map<string, float> AgvKinematics::pos = 
{
    {"LF_D", 0.0},
    {"RF_D", 0.0},
    {"LB_D", 0.0},
    {"RB_D", 0.0}
};

unordered_map<string, float> AgvKinematics::offsets =
{
    {"LF_D", 0.0},
    {"RF_D", 0.0},
    {"LB_D", 0.0},
    {"RB_D", 0.0}
};

MotorGoal AgvKinematics::natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    float trans_n = msg->vel_tau * sin(yaw_diff) + msg->vel_n * cos(yaw_diff);
    float trans_tao = msg->vel_tau * cos(yaw_diff) - msg->vel_n * sin(yaw_diff);
    float rot_vel = msg->omega * RATIO / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_n + rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RF", trans_n + rot_vel, trans_tao + rot_vel);
    add_group_goal(motor_goal, "LB", trans_n - rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RB", trans_n - rot_vel, trans_tao + rot_vel);

    return motor_goal;
}

MotorGoal AgvKinematics::absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    float trans_n = msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw);
    float trans_tao = msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw);
    float rot_vel = msg->omega * RATIO / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_n + rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RF", trans_n + rot_vel, trans_tao + rot_vel);
    add_group_goal(motor_goal, "LB", trans_n - rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RB", trans_n - rot_vel, trans_tao + rot_vel);
    
    return motor_goal;
}

MotorGoal AgvKinematics::chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg)
{
    MotorGoal motor_goal;
    clear_goal(motor_goal);

    float trans_n = msg->vel_y;
    float trans_tao = msg->vel_x;
    float rot_vel = msg->omega * RATIO / sqrt(2);

    add_group_goal(motor_goal, "LF", trans_n + rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RF", trans_n + rot_vel, trans_tao + rot_vel);
    add_group_goal(motor_goal, "LB", trans_n - rot_vel, trans_tao - rot_vel);
    add_group_goal(motor_goal, "RB", trans_n - rot_vel, trans_tao + rot_vel);
    
    return motor_goal;
}

void AgvKinematics::clear_goal(MotorGoal &motor_goal)
{
    motor_goal.motor_id.clear();
    motor_goal.goal_vel.clear();
    motor_goal.goal_pos.clear();
}

void AgvKinematics::add_motor_goal(MotorGoal &motor_goal, const string& rid, const float goal_vel, const float goal_pos)
{
    motor_goal.motor_id.push_back(rid);
    motor_goal.goal_vel.push_back(goal_vel);
    motor_goal.goal_pos.push_back(goal_pos + offsets[rid]);
}

void AgvKinematics::add_group_goal(MotorGoal &motor_goal, const string& which, float vx, float vy)
{
    string dir_id = which + "_D";
    string vel_id = which + "_V";

    float velocity = rss(vx, vy);
    if (is_zero(velocity)) // if vel is 0, keep the direction
    {
        // pos[dir_id] does not change
        vel[vel_id] = 0;
    }
    else if (is_zero(vy)) // if vy is 0, vx/vy would be 0/0
    {
        pos[dir_id] = PI / 2;
        vel[vel_id] = vy;
    }
    else // if vy is not 0, calculate the direction
    {
        pos[dir_id] = atan(vx / vy);
        vel[vel_id] = velocity;
    }
    add_motor_goal(motor_goal, vel_id, vel[vel_id], 0);
    add_motor_goal(motor_goal, dir_id, 0, pos[dir_id]);
    
}

float AgvKinematics::rss(float x, float y)
{
    return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

bool AgvKinematics::is_zero(float x)
{
    return std::abs(x) < TRIGGER;
}