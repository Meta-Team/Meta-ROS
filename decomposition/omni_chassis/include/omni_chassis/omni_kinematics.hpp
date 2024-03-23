#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <rclcpp/node.hpp>
#include <vector>
#include <iostream>

#include "behavior_interface/msg/move.hpp"
#include "motor_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

#define DIRE -1 // 1 if counter-clockwise, -1 if clockwise
#define PI 3.1415926f

/**
 * @brief This namespace contains functions to decompose movement commands into motor goals.
 */
namespace OmniKinematics
{
    extern float wheel_r; ///< The radius of the wheels, in meters.
    extern float cha_r; ///< The radius of the chassis, in meters.
    extern float decel_ratio; ///< The ratio of motor deceleration.
    extern float n_offset; ///< The feedback angle of ahrs, when the ahrs is facing the desired front.
    extern float yaw_offset; ///< The feedback angle of yaw motor, when gimbal is parallel to the chassis front.

    /**
     * @brief Decompose a absolute movement command into motor goals.
     * An absolute movement is a movement relative to the ground.
     * @param msg The absolute movement command.
     * @param gimbal The yaw position of the gimbal against the ground, in radians.
     * @param motor The yaw position of the gimbal against the chassis, in radians.
     * @return The motor goals.
     */
    motor_interface::msg::MotorGoal absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg,
                                                     float gimbal, float motor);

    /**
     * @brief Decompose a chassis movement command into motor goals.
     * A chassis movement is a movement relative to the chassis.
     * @param msg The chassis movement command.
     * @return The motor goals.
     * @note This is independent of any feedback and is recommended in testing.
     */
    motor_interface::msg::MotorGoal chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg);

    /**
     * @brief Decompose a natural movement command into motor goals.
     * A natural movement is a movement relative to the chassis and gimbal.
     * @param msg The natural movement command.
     * @param motor The yaw position of the gimbal against the chassis, in radians.
     * @return The motor goals.
     */
    motor_interface::msg::MotorGoal natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, float motor);

    /**
     * @brief Clear the motor goals.
     * @param motor_goal The motor goals to be cleared.
     * @return The DM goal.
     */
    void clear_goal(motor_interface::msg::MotorGoal &motor_goal);

    /**
     * @brief Add the goal velocity and position of a motor.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_vel The goal velocity of the wheels in m/s.
     * @note Goal positions would be set zero.
     */
    void add_goal(motor_interface::msg::MotorGoal &motor_goals,
                  const std::string& rid, const float goal_vel);
}

#endif // OMNI_KINEMATICS_HPP