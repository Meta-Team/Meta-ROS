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

/**
 * @brief Namespace containing functions and types related to AGV kinematics.
 *
 * An AGV chassis has 4 direction motors and 4 velocity motors.
 * The direction motors are controlled by DM motor drivers, while the velocity motors are controlled by DJI motor drivers.
 *
 * @note In this namespace, left is set to be positive and forward is set to be positive.    
 */
namespace AgvKinematics
{
    /**
    * @brief Decomposes a natural move message into DM motor goals.
    * @param msg The natural move message.
    * @param yaw_diff The yaw difference.
    * @return The DM motor goal.
    */
    motor_interface::msg::DmGoal natural_dm_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    /**
    * @brief Decomposes an absolute move message into DM motor goals.
    * @param msg The absolute move message.
    * @param chassis_yaw The chassis yaw.
    * @return The DM motor goal.
    */
    motor_interface::msg::DmGoal absolute_dm_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    /**
    * @brief Decomposes a natural move message into DJI motor goals.
    * @param msg The natural move message.
    * @param yaw_diff The yaw difference.
    * @return The DJI motor goal.
    */
    motor_interface::msg::DjiGoal natural_dji_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    /**
    * @brief Decomposes an absolute move message into DJI motor goals.
    * @param msg The absolute move message.
    * @param chassis_yaw The chassis yaw.
    * @return The DJI motor goal.
    */
    motor_interface::msg::DjiGoal absolute_dji_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    /**
    * @brief Adds a DM motor goal to the existing motor goals.
    * @param motor_goals The existing motor goals.
    * @param motor_id The motor ID.
    * @param goal_vel The goal velocity.
    * @param goal_pos The goal position.
    */
    void add_dm_goal(motor_interface::msg::DmGoal &motor_goals, const DirectionMotor motor_id, const float goal_vel, const float goal_pos);

    /**
    * @brief Sets a DJI motor goal.
    * @param motor_goals The motor goals.
    * @param motor_id The motor ID.
    * @param goal_pos The goal position.
    */
    void set_dji_goal(motor_interface::msg::DjiGoal &motor_goals, const VelocityMotor motor_id, const float goal_pos);

    /**
    * @brief Clears the DM motor goals.
    * @param motor_goals The motor goals.
    */
    void clear_dm_goals(motor_interface::msg::DmGoal &motor_goals);

    /**
    * @brief Calculates the root sum square of two values.
    * @param x The first value.
    * @param y The second value.
    * @return The root sum square.
    */
    float rss(float x, float y);
}

#endif // AGV_KINEMATICS_HPP