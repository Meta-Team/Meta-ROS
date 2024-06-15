#ifndef OMNI_KINEMATICS_HPP
#define OMNI_KINEMATICS_HPP

#include <cmath>
#include <rclcpp/node.hpp>
#include <vector>
#include <iostream>

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

#define DIR -1 // 1 if counter-clockwise, -1 if clockwise

using device_interface::msg::MotorGoal;

/**
 * @brief This namespace contains functions to decompose movement commands into motor goals.
 */
class OmniKinematics
{
private:
    double wheel_r; ///< The radius of the wheels, in meters.
    double cha_r; ///< The radius of the chassis, in meters.
    double decel_ratio; ///< The ratio of motor deceleration.
    double n_offset; ///< The feedback angle of ahrs, when the ahrs is facing the desired front.
    double yaw_offset; ///< The feedback angle of yaw motor, when gimbal is parallel to the chassis front.

    MotorGoal motor_goal; ///< The motor goals to be sent.
    double last_rec = 0.0; ///< The last time a message was received.
    static MotorGoal stop; ///< The goal to stop all motors on the chassis.

public:
    /**
     * @brief Constructor of the OmniKinematics class.
     * @param wheel_r The radius of the wheels, in meters.
     * @param cha_r The radius of the chassis, in meters.
     * @param decel_ratio The ratio of motor deceleration.
     * @param n_offset The feedback angle of ahrs, when the ahrs is facing the desired front.
     * @param yaw_offset The feedback angle of yaw motor, when gimbal is parallel to the chassis front.
     */
    OmniKinematics(double wheel_r, double cha_r, double decel_ratio, double n_offset, double yaw_offset);

    /**
     * @brief Decompose a absolute movement command into motor goals.
     * An absolute movement is a movement relative to the ground.
     * @param msg The absolute movement command.
     * @param gimbal The yaw position of the gimbal against the ground, in radians.
     * @param motor The yaw position of the gimbal against the chassis, in radians.
     * @return The motor goals.
     */
    void absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, double gimbal, double motor);

    /**
     * @brief Decompose a chassis movement command into motor goals.
     * A chassis movement is a movement relative to the chassis.
     * @param msg The chassis movement command.
     * @return The motor goals.
     * @note This is independent of any feedback and is recommended in testing.
     */
    void chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg);

    /**
     * @brief Decompose a natural movement command into motor goals.
     * A natural movement is a movement relative to the chassis and gimbal.
     * @param msg The natural movement command.
     * @param motor The yaw position of the gimbal against the chassis, in radians.
     * @return The motor goals.
     */
    void natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double motor);

    /**
     * @brief Get the motor goals.
     * @return The motor goals to be sent.
     * @note Return a stop goal if the chassis is not active.
     */
    MotorGoal get_motor_goal() const;

private:
    /**
     * @brief Clear the motor goals.
     * @param motor_goal The motor goals to be cleared.
     * @return The DM goal.
     */
    static void clear_goal(MotorGoal &motor_goal);

    /**
     * @brief Add the goal velocity and position of a motor.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_vel The goal velocity of the wheels in m/s.
     * @note Goal positions would be set zero.
     */
    void add_goal(MotorGoal &motor_goals, const std::string& rid, const double goal_vel);
};

#endif // OMNI_KINEMATICS_HPP