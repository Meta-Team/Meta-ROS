#ifndef AGV_KINEMATICS_HPP
#define AGV_KINEMATICS_HPP

#include <cmath>
#include <map>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <thread>

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
class AgvKinematics
{
private:
    double wheel_r; ///< The radius of the wheels, in meters.
    double cha_r; ///< The radius of the chassis, in meters.
    double decel_ratio; ///< The ratio of motor deceleration.
    double n_offset; ///< The feedback angle of ahrs, when the ahrs is facing the desired front.
    double yaw_offset; ///< The feedback angle of yaw motor, when gimbal is parallel to the chassis front.

    unordered_map<string, double> vel; ///< The velocities of the motors.
    unordered_map<string, pair<int, double>> pos; ///< The positions of the motors.
    unordered_map<string, double> offsets; ///< The offsets of the motors.

    MotorGoal motor_goal; ///< The motor goals to be sent.
    double last_rec = 0.0; ///< The last time a message was received.
    static MotorGoal stop; ///< The goal to stop all motors on the chassis.

public:
    /**
     * @brief Constructor of the AgvKinematics class.
     * @param wheel_r The radius of the wheels, in meters.
     * @param cha_r The radius of the chassis, in meters.
     * @param decel_ratio The ratio of motor deceleration.
     * @param n_offset The feedback angle of ahrs, when the ahrs is facing the desired front.
     * @param yaw_offset The feedback angle of yaw motor, when gimbal is parallel to the chassis front.
     */
    AgvKinematics(double wheel_r, double cha_r, double decel_ratio, double n_offset, double yaw_offset);

    /**
     * @brief Set the offsets of the motors.
     * @param lf The offset of the left front motor, in radians.
     * @param rf The offset of the right front motor, in radians.
     * @param lb The offset of the left back motor, in radians.
     * @param rb The offset of the right back motor, in radians.
     */
    void set_offsets(double lf, double rf, double lb, double rb);

    /**
     * @brief Decomposes a natural move message into motor goals.
     * @param msg The natural move message.
     * @param yaw_diff The yaw difference between chassis and gimbal.
     */
    void natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double yaw_diff);

    /**
     * @brief Decomposes an absolute move message into motor goals.
     * @param msg The absolute move message.
     * @param gimbal The ahrs yaw position.
     * @param motor The yaw motor position.
     */
    void absolute_decompo(const behavior_interface::msg::Move::SharedPtr msg, double gimbal, double motor);

    /**
     * @brief Decomposes a chassis move message into motor goals.
     * @param msg The chassis move message.
     */
    void chassis_decompo(const behavior_interface::msg::Move::SharedPtr msg);

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
    static double rss(double x, double y);

    /**
     * @brief Checks if a double is close to zero.
     * @param x The double to be checked.
     * @return True if the double is zero, false otherwise.
     */
    static bool is_zero(double x);

    /**
     * @brief Calculates the minimum difference between two angles.
     * This function takes two angles a and b (in radians) and calculates the difference between them.
     * The result is adjusted to be in the range of [-M_PI, M_PI].
     * @param a The first angle in radians.
     * @param b The second angle in radians.
     * @return The minimum difference between a and b in the range of [-M_PI, M_PI].
     */
    static double min_diff(const double& a, const double& b);

    /**
     * @brief Normalizes the angle of a position.
     * This function takes a position represented as a pair of round and angle.
     * It adjusts the angle to be in the range of [0, 2 * M_PI] and updates the round accordingly. 
     * @param pos The position to be normalized, represented as a pair of round and angle.
     */
    static void normalize(pair<int, double>& pos);

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
    static tuple<int, double, bool> closest_angle(const double& new_goal, const pair<int, double>& prev_pos);
};

#endif // AGV_KINEMATICS_HPP