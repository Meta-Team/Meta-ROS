#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#include <device_interface/msg/detail/motor_goal__struct.hpp>
#include <queue>
#include <thread>
#include "uni_gimbal/pid_algorithm.hpp"

#include "device_interface/msg/motor_goal.hpp"

#define IMU_FB true

#define CALC_FREQ 1 // ms

#define NaN std::nan("")

using device_interface::msg::MotorGoal;

struct PidParam
{
    double kp = 0.0; ///< The proportional gain of the PID controller.
    double ki = 0.0; ///< The integral gain of the PID controller.
    double kd = 0.0; ///< The derivative gain of the PID controller.

    PidParam(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd) {}
    PidParam() : kp(0), ki(0), kd(0) {}
};

/**
 * @brief A class representing a gimbal.
 */
class Gimbal
{
public:
#if IMU_FB == false
    /**
     * @brief Construct a new Gimbal object.
     * @param yaw_p2v The position-to-velocity PID parameters for the yaw axis.
     * @param pitch_p2v The position-to-velocity PID parameters for the pitch axis.
     */
    Gimbal(PidParam yaw_p2v, PidParam pitch_p2v);
#else // IMU_FB == true
    /**
     * @brief Construct a new Gimbal object.
     * @param yaw_p2v The position-to-voltage PID parameters for the yaw axis.
     * @param pitch_p2v The position-to-voltage PID parameters for the pitch axis.
     * @param yaw_v2v The velocity-to-voltage PID parameters for the yaw axis.
     * @param pitch_v2v The velocity-to-voltage PID parameters for the pitch axis.
     */
    Gimbal(PidParam yaw_p2v, PidParam pitch_p2v, PidParam yaw_v2v, PidParam pitch_v2v);
#endif // IMU_FB

    /**
     * @brief Set the goal position of the gimbal.
     * @param goal_yaw_pos The goal yaw position of the gimbal, in radians, relative to north.
     * @param goal_pitch_pos The goal pitch position of the gimbal, in radians.
     */
    void set_goal(double goal_yaw_pos, double goal_pitch_pos);

    /**
     * @brief Update the position feedback from ahrs.
     * @param yaw_pos The current yaw position of the gimbal, in radians, relative to north.
     * @param pitch_pos The current pitch position of the gimbal, in radians.
     */
    void update_pos_feedback(double yaw_pos, double pitch_pos);

#if IMU_FB == true
    /**
     * @brief Update the velocity feedback from imu.
     * @param yaw_vel The current yaw velocity of the gimbal, in rad/s.
     * @param pitch_vel The current pitch velocity of the gimbal, in rad/s.
     */
    void update_vel_feedback(double yaw_vel, double pitch_vel);
#endif // IMU_FB

    /**
     * @brief Calculate the minimum error in (-M_PI, M_PI] between the goal and the current position.
     * Similar to `goal - current`, but in the range (-M_PI, M_PI].
     * @param goal The goal position.
     * @param current The current position.
     * @return The minimum error.
     * @note Both goal and current should be in [0, 2*M_PI).
     */
    double min_error(double goal, double current);

    /**
     * @brief Get the motor goals.
     * @return The motor goals to be sent.
     */
    [[nodiscard]] MotorGoal get_motor_goal() const;

private:
    double last_rec = 0.0; ///< The last time a message was received.
    static MotorGoal stop; ///< The goal to stop all motors on the chassis.

    std::unique_ptr<PidAlgorithm> yaw_p2v; ///< The yaw position-to-velocity controller, using cumulated position.
    std::unique_ptr<PidAlgorithm> pitch_p2v; ///< The pitch position-to-velocity controller.
    std::unique_ptr<PidAlgorithm> yaw_v2v; ///< The yaw velocity-to-voltage controller.
    std::unique_ptr<PidAlgorithm> pitch_v2v; ///< The pitch velocity-to-voltage controller.

#if IMU_FB == true
    std::thread vel_thread; ///< The thread for passing velocity goals from p2v to v2v.

    void vel_loop(); ///< The loop for passing velocity goals from p2v to v2v.
#endif // IMU_FB
};

#endif // GIMBAL_PID_H