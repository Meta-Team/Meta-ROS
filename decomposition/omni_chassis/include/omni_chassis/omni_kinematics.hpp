#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <motor_interface/msg/detail/dji_goal__struct.hpp>
#include <rclcpp/node.hpp>
#include <vector>
#include <iostream>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "movement_interface/msg/chassis_move.hpp"
#include "motor_interface/msg/dji_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/srv/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter

enum WheelId
{
    F = 1,
    L = 2,
    B = 3,
    R = 4,
};
// all mounted counter-clockwise

/**
 * @brief This namespace contains functions to decompose movement commands into motor goals.
 */
namespace OmniKinematics
{
    /**
     * @brief Decompose a absolute movement command into motor goals.
     * An absolute movement is a movement relative to the ground.
     * @param msg The absolute movement command.
     * @return The motor goals.
     * @param chassis_yaw The yaw angle of the chassis.
     */
    motor_interface::msg::DjiGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    /**
     * @brief Decompose a chassis movement command into motor goals.
     * A chassis movement is a movement relative to the chassis.
     * @param msg The chassis movement command.
     * @return The motor goals.
     * @note This is independent of any feedback and is recommended in testing.
     */
    motor_interface::msg::DjiGoal chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg);

    /**
     * @brief Set the goal velocity and position of a motor.
     * Set position to 0 to control the motor in velocity mode.
     * Set velocity to 0 to control the motor in position mode.
     * @param[out] motor_goals The motor goals to be set.
     * @param wheel_id The id of the motor.
     * @param goal_vel The goal velocity of the motor.
     * @param goal_pos The goal position of the motor.
     * @param order The order in the motor goal arrays, from 0 to 3. Different motors can exchange their orders.
     */
    void set_goal(motor_interface::msg::DjiGoal &motor_goals,
                  const WheelId wheel_id, const float goal_vel, const float goal_pos,
                  int order);
};

#endif // OMNI_KINEMATICS_HPP