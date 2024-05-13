#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

#define TRIGGER 0.2 // a vel is considered to be zero if its absolute value is less than TRIGGER

#define DIR -1 // -1 if direction motors take clockwise as positive, 1 if counterclockwise

using std::string, std::unordered_map, std::pair, std::tuple;
using device_interface::msg::MotorGoal;

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
    extern unordered_map<string, pair<int, double>> pos; ///< The positions of the motors.
    extern unordered_map<string, double> offsets; ///< The offsets of the motors.
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

    /**
     * @brief Calculates the minimum difference between two angles.
     * This function takes two angles a and b (in radians) and calculates the difference between them.
     * The result is adjusted to be in the range of [-M_PI, M_PI].
     * @param a The first angle in radians.
     * @param b The second angle in radians.
     * @return The minimum difference between a and b in the range of [-M_PI, M_PI].
     */
    double min_diff(const double& a, const double& b);

    /**
     * @brief Normalizes the angle of a position.
     * This function takes a position represented as a pair of round and angle.
     * It adjusts the angle to be in the range of [0, 2 * M_PI] and updates the round accordingly. 
     * @param pos The position to be normalized, represented as a pair of round and angle.
     */
    void normalize(pair<int, double>& pos);

    /**
     * @brief Calculates the closest angle to a new goal from a previous position.
     * This function takes a new goal angle and a previous position (round and angle),
     * and calculates the closest angle to the new goal from the previous position.
     * It also determines whether a reverse is needed based on the difference between the previous angle and the new goal.
     * @param new_goal The new goal angle in radians.
     * @param prev_pos The previous position, represented as a pair of [round, angle].
     * @return A tuple [round, angle, reverse] containing the round, the closest angle to the new goal,
     *         and a boolean indicating whether a reverse is needed.
     */
    tuple<int, double, bool> closest_angle(const double& new_goal, const pair<int, double>& prev_pos);
}

#endif // AGV_KINEMATICS_HPP