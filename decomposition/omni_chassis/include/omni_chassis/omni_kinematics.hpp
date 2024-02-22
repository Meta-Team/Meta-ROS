#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <rclcpp/node.hpp>
#include <vector>
#include <iostream>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "movement_interface/msg/chassis_move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/srv/motor_present.hpp"

#define DIRE -1 // 1 if counter-clockwise, -1 if clockwise
#define PI 3.14159265358979323846
#define RADIUS 1 // meter

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
    motor_interface::msg::MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg,
                                                     float chassis_yaw);

    /**
     * @brief Decompose a chassis movement command into motor goals.
     * A chassis movement is a movement relative to the chassis.
     * @param msg The chassis movement command.
     * @return The motor goals.
     * @note This is independent of any feedback and is recommended in testing.
     */
    motor_interface::msg::MotorGoal chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg);

    /**
     * @brief Clear the motor goals.
     * @param motor_goal The motor goals to be cleared.
     * @return The DM goal.
     */
    void clear_goal(motor_interface::msg::MotorGoal &motor_goal);

    /**
     * @brief Add the goal velocity and position of a motor.
     * Set position to 0 to control the motor in velocity mode.
     * Set velocity to 0 to control the motor in position mode.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_vel The goal velocity of the motor.
     * @param goal_pos The goal position of the motor.
     */
    void add_goal(motor_interface::msg::MotorGoal &motor_goals,
                  const std::string& rid, const float goal_vel, const float goal_pos);
}

#endif // OMNI_KINEMATICS_HPP