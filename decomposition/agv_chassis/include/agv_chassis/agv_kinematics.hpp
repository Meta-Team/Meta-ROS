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
#define RATIO 1 // ratio between velocity and angular velocity
#define TRIGGER 0.1 // a vel is considered to be zero if its absolute value is less than TRIGGER

using std::string, std::unordered_map, motor_interface::msg::MotorGoal;

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
    extern unordered_map<string, float> vel; ///< The velocities of the motors.
    extern unordered_map<string, float> pos; ///< The positions of the motors.
    extern unordered_map<string, float> offsets; ///< The offsets of the motors. Used in add_motor_goal.

    /**
     * @brief Decomposes a natural move message into motor goals.
     * @param msg The natural move message.
     * @param yaw_diff The yaw difference.
     * @return The motor goal.
     */
    MotorGoal natural_decompo(const movement_interface::msg::NaturalMove::SharedPtr msg, float yaw_diff);

    /**
     * @brief Decomposes an absolute move message into motor goals.
     * @param msg The absolute move message.
     * @param chassis_yaw The chassis yaw.
     * @return The motor goal.
     */
    MotorGoal absolute_decompo(const movement_interface::msg::AbsoluteMove::SharedPtr msg, float chassis_yaw);

    /**
     * @brief Decomposes a chassis move message into motor goals.
     * @param msg The chassis move message.
     * @return The motor goal.
     */
    MotorGoal chassis_decompo(const movement_interface::msg::ChassisMove::SharedPtr msg);

    /**
     * @brief Clear the motor goals.
     * @param motor_goal The motor goals to be cleared.
     * @return The DM goal.
     */
    void clear_goal(MotorGoal &motor_goal);

    /**
     * @brief Add the goal velocity and position of a motor.
     * Set position to 0 to control the motor in velocity mode.
     * Set velocity to 0 to control the motor in position mode.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_vel The goal velocity of the motor.
     * @param goal_pos The goal position of the motor.
     */
    void add_motor_goal(MotorGoal &motor_goal, const std::string& rid, const float goal_vel, const float goal_pos);

    /**
     * @brief Add the goal velocity and position of the two motors of a wheel.
     * @param[out] motor_goals The motor goals to be set.
     * @param which Which wheel to be set. Must be "LF", "RF", "LB", or "RB".
     * @param vx The x component against the global frame.
     * @param vy The y component against the global frame.
     */
    void add_group_goal(MotorGoal &motor_goal, const string& which, float vx, float vy);

    /**
     * @brief Calculates the root sum square of two values.
     * @param x The first value.
     * @param y The second value.
     * @return The root sum square.
     */
    float rss(float x, float y);

    /**
     * @brief Checks if a float is close to zero.
     * @param x The float to be checked.
     * @return True if the float is zero, false otherwise.
     */
    bool is_zero(float x);
};

#endif // AGV_KINEMATICS_HPP