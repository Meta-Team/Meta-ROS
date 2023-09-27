#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <motor_interface/msg/detail/dm_goal__struct.hpp>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "motor_interface/msg/dm_goal.hpp"
#include "motor_interface/msg/dji_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/srv/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter

enum DirectionMotor
{
    LF_D,
    RF_D,
    LB_D,
    RB_D
};

enum VelocityMotor
{
    LF_V,
    RF_V,
    LB_V,
    RB_V
};

namespace AgvKinematics
{
    motor_interface::msg::DmGoal natural_dm_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    motor_interface::msg::DmGoal absolute_dm_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    motor_interface::msg::DjiGoal natural_dji_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    motor_interface::msg::DjiGoal absolute_dji_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    void add_dm_goal(motor_interface::msg::DmGoal &motor_goals, const DirectionMotor motor_id, const float goal_vel, const float goal_pos);

    void set_dji_goal(motor_interface::msg::DjiGoal &motor_goals, const VelocityMotor motor_id, const float goal_pos);

    void clear_dm_goals(motor_interface::msg::DmGoal &motor_goals);

    float rss(float x, float y); // root sum square
};

#endif // AGV_KINEMATICS_HPP