#ifndef MECANUM_KINEMATICS_HPP
#define MECANUM_KINEMATICS_HPP

#include <cmath>
#include <vector>
#include <iostream>

#include "behavior_interface/msg/move.hpp"
#include "motor_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

#define CALC_FREQ 1 // ms
#define OMEGA_MAX 6
#define DT CALC_FREQ / 1000.0 // s

struct PidParam
{
    double kp = 0.0; ///< The proportional gain of the PID controller.
    double ki = 0.0; ///< The integral gain of the PID controller.
    double kd = 0.0; ///< The derivative gain of the PID controller.

    PidParam(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}
    PidParam() : kp(0), ki(0), kd(0) {}
};

struct PidOutput
{
    double p = 0.0; ///< The proportional term of the PID controller.
    double i = 0.0; ///< The integral term of the PID controller.
    double d = 0.0; ///< The derivative term of the PID controller.

    double sum() { return p + i + d; }
};

namespace MecanumKinematics
{
    extern double cha_wid; ///< The width of the chassis, in meters.
    extern double cha_len; ///< The length of the chassis, in meters.
    extern double wheel_angle; ///< The angle of the small wheels, in radians.
    extern double wheel_r; ///< The radius of the wheels, in meters.
    extern double decel_ratio; ///< The ratio of motor deceleration.
    extern double yaw_offset; ///< The feedback angle of yaw motor, when gimbal is parallel to the chassis front.

    extern PidParam cha_param; ///< The PID parameters for the chassis.
    extern PidOutput cha_output; ///< The PID output for the chassis.

    extern double yaw_error; ///< The error of the yaw motor.

    /**
     * @brief Decompose a chassis movement command into motor goals.
     * A chassis movement is a movement relative to the chassis.
     * @param v_x The velocity of the chassis in the x-axis, in m/s.
     * @param v_y The velocity of the chassis in the y-axis, in m/s.
     * @param omega The angular velocity of the chassis, in rad/s.
     * @return The motor goals.
     * @note This is independent of any feedback and is recommended in testing.
     */
    motor_interface::msg::MotorGoal chassis_decompo(const double v_x, const double v_y, const double omega);

    /**
     * @brief Decompose a natural movement command into motor goals.
     * A natural movement is a movement relative to the chassis and gimbal.
     * @param msg The natural movement command.
     * @param motor The yaw position of the gimbal against the chassis, in radians.
     * @return The motor goals.
     */
    motor_interface::msg::MotorGoal natural_decompo(const behavior_interface::msg::Move::SharedPtr msg, double motor);

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
                  const std::string& rid, const double goal_vel);
}

#endif // MECANUM_KINEMATICS_HPP