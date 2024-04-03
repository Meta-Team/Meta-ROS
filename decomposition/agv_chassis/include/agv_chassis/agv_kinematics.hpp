#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_interface/msg/move.hpp"
#include "motor_interface/msg/motor_goal.hpp"
#include "motor_interface/srv/motor_state.hpp"

#define NaN std::nan("")

#define TRIGGER 0.05 // a vel is considered to be zero if its absolute value is less than TRIGGER

using std::string, std::unordered_map, std::pair, motor_interface::msg::MotorGoal;

/**
 * @brief Namespace containing functions and types related to AGV kinematics.
 * An AGV chassis has 4 direction motors and 4 velocity motors.
 * @note +x is the front of the chassis, +y is the left of the chassis.    
 */
namespace AgvKinematics
{
    extern double wheel_r; ///< The radius of the wheels, in meters.
    extern double cha_r; ///< The radius of the chassis, in meters.
    extern double decel_ratio; ///< The ratio of motor deceleration.
    extern double n_offset; ///< The feedback angle of ahrs, when the ahrs is facing the desired front.
    extern double yaw_offset; ///< The feedback angle of yaw motor, when gimbal is parallel to the chassis front.

    extern unordered_map<string, double> vel; ///< The velocities of the motors.
    extern unordered_map<string, double> pos; ///< The positions of the motors.
    extern unordered_map<string, pair<double, bool>> offsets; ///< The offsets of the motors. Bool indicates whether the offset is found.

    /**
     * @brief Decomposes a natural move message into motor goals.
     * @param msg The natural move message.
     * @param yaw_diff The yaw difference between chassis and gimbal.
     * @return The motor goal.
     */
    MotorGoal natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double yaw_diff);

    /**
     * @brief Decomposes an absolute move message into motor goals.
     * @param msg The absolute move message.
     * @param gimbal The ahrs yaw position.
     * @param motor The yaw motor position.
     * @return The motor goal.
     */
    MotorGoal absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, double gimbal, double motor);

    /**
     * @brief Decomposes a chassis move message into motor goals.
     * @param msg The chassis move message.
     * @return The motor goal.
     */
    MotorGoal chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg);

    /**
     * @brief Clear the motor goals.
     * @param motor_goal The motor goals to be cleared.
     * @return The DM goal.
     */
    void clear_goal(MotorGoal &motor_goal);

    /**
     * @brief Add the goal velocity and position of a motor.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_vel The goal velocity of the wheels in m/s.
     * @note Goal positions would be set zero.
     */
    void add_vel_goal(MotorGoal &motor_goal, const string& rid, const double goal_vel);

    /**
     * @brief Add the goal position of a motor.
     * @param[out] motor_goals The motor goals to be set.
     * @param rid The ROS id of the motor.
     * @param goal_pos The goal position of the wheels in rad.
     * @note Goal velocities would be set zero.
     */
    void add_pos_goal(MotorGoal &motor_goal, const string& rid, const double goal_pos);

    /**
     * @brief Add the goal velocity and position of the two motors of a wheel.
     * @param[out] motor_goals The motor goals to be set.
     * @param which Which wheel to be set. Must be "LF", "RF", "LB", or "RB".
     * @param vx The front component against the chassis frame in m/s.
     * @param vy The left component against the global frame in m/s.
     */
    void add_group_goal(MotorGoal &motor_goal, const string& which, double vx, double vy);

    /**
     * @brief Calculates the root sum square of two values.
     * @param x The first value.
     * @param y The second value.
     * @return The root sum square.
     */
    double rss(double x, double y);

    /**
     * @brief Checks if a double is close to zero.
     * @param x The double to be checked.
     * @return True if the double is zero, false otherwise.
     */
    bool is_zero(double x);
}

#endif // AGV_KINEMATICS_HPP