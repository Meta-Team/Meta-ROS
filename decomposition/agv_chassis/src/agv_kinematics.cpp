#include "agv_chassis/agv_kinematics.hpp"
#include <cmath>
#include <math.h>

motor_interface::msg::DmGoal AgvKinematics::natural_dm_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff)
{
    motor_interface::msg::DmGoal dm_goal;
    float trans_n = msg->vel_tau * sin(yaw_diff) + msg->vel_n * cos(yaw_diff);
    float trans_tao = msg->vel_tau * cos(yaw_diff) - msg->vel_n * sin(yaw_diff);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    clear_dm_goals(dm_goal);
    add_dm_goal(dm_goal, LF_D, atan((trans_n + rot_vel) / (trans_tao - rot_vel)), 0);
    add_dm_goal(dm_goal, RF_D, atan((trans_n + rot_vel) / (trans_tao + rot_vel)), 0);
    add_dm_goal(dm_goal, LB_D, atan((trans_n - rot_vel) / (trans_tao - rot_vel)), 0);
    add_dm_goal(dm_goal, RB_D, atan((trans_n - rot_vel) / (trans_tao + rot_vel)), 0);

    return dm_goal;
}

motor_interface::msg::DjiGoal AgvKinematics::natural_dji_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff)
{
    motor_interface::msg::DjiGoal dji_goal;
    float trans_n = msg->vel_tau * sin(yaw_diff) + msg->vel_n * cos(yaw_diff);
    float trans_tao = msg->vel_tau * cos(yaw_diff) - msg->vel_n * sin(yaw_diff);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    set_dji_goal(dji_goal, LF_V, rss(trans_n + rot_vel, trans_tao - rot_vel));
    set_dji_goal(dji_goal, RF_V, rss(trans_n + rot_vel, trans_tao + rot_vel));
    set_dji_goal(dji_goal, LB_V, rss(trans_n - rot_vel, trans_tao - rot_vel));
    set_dji_goal(dji_goal, RB_V, rss(trans_n - rot_vel, trans_tao + rot_vel));

    return dji_goal;
}

motor_interface::msg::DmGoal AgvKinematics::absolute_dm_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::DmGoal dm_goal;
    float trans_n = msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw);
    float trans_tao = msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    clear_dm_goals(dm_goal);
    add_dm_goal(dm_goal, LF_D, atan((trans_n + rot_vel) / (trans_tao - rot_vel)), 0);
    add_dm_goal(dm_goal, RF_D, atan((trans_n + rot_vel) / (trans_tao + rot_vel)), 0);
    add_dm_goal(dm_goal, LB_D, atan((trans_n - rot_vel) / (trans_tao - rot_vel)), 0);
    add_dm_goal(dm_goal, RB_D, atan((trans_n - rot_vel) / (trans_tao + rot_vel)), 0);

    return dm_goal;
}

motor_interface::msg::DjiGoal AgvKinematics::absolute_dji_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw)
{
    motor_interface::msg::DjiGoal dji_goal;
    float trans_n = msg->vel_x * cos(chassis_yaw) - msg->vel_y * sin(chassis_yaw);
    float trans_tao = msg->vel_x * sin(chassis_yaw) + msg->vel_y * cos(chassis_yaw);
    float rot_vel = msg->omega * RADIUS / sqrt(2);

    set_dji_goal(dji_goal, LF_V, rss(trans_n + rot_vel, trans_tao - rot_vel));
    set_dji_goal(dji_goal, RF_V, rss(trans_n + rot_vel, trans_tao + rot_vel));
    set_dji_goal(dji_goal, LB_V, rss(trans_n - rot_vel, trans_tao - rot_vel));
    set_dji_goal(dji_goal, RB_V, rss(trans_n - rot_vel, trans_tao + rot_vel));

    return dji_goal;
}

void AgvKinematics::add_dm_goal(motor_interface::msg::DmGoal &motor_goals, const DirectionMotor motor_id, const float goal_vel, const float goal_pos)
{
    motor_goals.motor_id.push_back(motor_id);
    motor_goals.goal_vel.push_back(goal_vel);
    motor_goals.goal_pos.push_back(goal_pos);
}

void AgvKinematics::clear_dm_goals(motor_interface::msg::DmGoal &motor_goals)
{
    motor_goals.motor_id.clear();
    motor_goals.goal_pos.clear();
    motor_goals.goal_vel.clear();
}

float AgvKinematics::rss(float x, float y)
{
    return std::sqrt(x * x + y * y);
}

void AgvKinematics::set_dji_goal(motor_interface::msg::DjiGoal &motor_goals, const VelocityMotor motor_id, const float goal_pos)
{
    motor_goals.motor_id[motor_id] = motor_id;
    motor_goals.goal_pos[motor_id] = goal_pos;
    motor_goals.goal_vel[motor_id] = 0;
}