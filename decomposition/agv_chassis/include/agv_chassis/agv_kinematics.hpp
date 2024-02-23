#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <map>
#include <unordered_map>
#include <vector>

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"
#include "movement_interface/msg/chassis_move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "gyro_interface/srv/gimbal_position.hpp"
#include "motor_interface/srv/motor_present.hpp"

#define PI 3.14159265358979323846
#define RADIUS 0.1 // meter

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
     * @brief The offsets of the motors.
     * Used to compensate the difference between the actual and the theoretical angles of the motors.
     */
    extern std::unordered_map<std::string, float> offsets;

    /**
     * @brief Decomposes a natural move message into motor goals.
     * @param msg The natural move message.
     * @param yaw_diff The yaw difference.
     * @return The motor goal.
     */
    motor_interface::msg::MotorGoal natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    /**
     * @brief Decomposes an absolute move message into motor goals.
     * @param msg The absolute move message.
     * @param chassis_yaw The chassis yaw.
     * @return The motor goal.
     */
    motor_interface::msg::MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    /**
     * @brief Decomposes a chassis move message into motor goals.
     * @param msg The chassis move message.
     * @return The motor goal.
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
    void add_goal(motor_interface::msg::MotorGoal &motor_goal, const std::string& rid, const float goal_vel, const float goal_pos);

    /**
     * @brief Calculates the root sum square of two values.
     * @param x The first value.
     * @param y The second value.
     * @return The root sum square.
     */
    float rss(float x, float y);
}

#endif // AGV_KINEMATICS_HPP