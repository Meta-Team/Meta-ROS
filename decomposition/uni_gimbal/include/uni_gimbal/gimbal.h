#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#include <queue>
#include <thread>

#define IMU_FB false

#define CALC_FREQ 1 // ms
#define V_MAX 12 // to be tuned, velocity limit
#define DT CALC_FREQ / 1000.0 // s

struct PidParam
{
    float kp = 0.0; ///< The proportional gain of the PID controller.
    float ki = 0.0; ///< The integral gain of the PID controller.
    float kd = 0.0; ///< The derivative gain of the PID controller.

    PidParam(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}
    PidParam() : kp(0), ki(0), kd(0) {}
};

struct PidOutput
{
    float p = 0.0; ///< The proportional term of the PID controller.
    float i = 0.0; ///< The integral term of the PID controller.
    float d = 0.0; ///< The derivative term of the PID controller.

    float sum() { return p + i + d; }
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
     * @param comp The ratio of compensate omega.
     */
    Gimbal(PidParam yaw_p2v, PidParam pitch_p2v, float comp);
#else // IMU_FB == true
    /**
     * @brief Construct a new Gimbal object.
     * @param yaw_p2v The position-to-voltage PID parameters for the yaw axis.
     * @param pitch_p2v The position-to-voltage PID parameters for the pitch axis.
     * @param yaw_v2v The velocity-to-voltage PID parameters for the yaw axis.
     * @param pitch_v2v The velocity-to-voltage PID parameters for the pitch axis.
     * @param comp The ratio of compensate omega.
     */
    Gimbal(PidParam yaw_p2v, PidParam pitch_p2v, PidParam yaw_v2v, PidParam pitch_v2v, float comp);
#endif // IMU_FB

    /**
     * @brief Destroy the Gimbal object.
     */
    ~Gimbal() { calc_thread.join(); }

    /**
     * @brief Set the goal position of the gimbal.
     * @param goal_yaw_pos The goal yaw position of the gimbal, in radians, relative to north.
     * @param goal_pitch_pos The goal pitch position of the gimbal, in radians.
     */
    void set_goal(float goal_yaw_pos, float goal_pitch_pos);

    /**
     * @brief Update the angular velocity of the chassis.
     * @param omega The angular velocity of the chassis, in rad/s. Positive for counterclockwise.
     */
    void update_omega(float omega);

    /**
     * @brief Update the position feedback from ahrs.
     * @param yaw_pos The current yaw position of the gimbal, in radians, relative to north.
     * @param pitch_pos The current pitch position of the gimbal, in radians.
     */
    void update_pos_feedback(float yaw_pos, float pitch_pos);

#if IMU_FB == true
    /**
     * @brief Update the velocity feedback from imu.
     * @param yaw_vel The current yaw velocity of the gimbal, in rad/s.
     * @param pitch_vel The current pitch velocity of the gimbal, in rad/s.
     */
    void update_vel_feedback(float yaw_vel, float pitch_vel);
#endif // IMU_FB

    /**
     * @brief Curb the value to the range of [-max, max].
     * @param val The value to curb.
     * @param max The maximum value.
     */
    void curb(float &val, float max);

    /**
     * @brief Calculate the minimum error between the goal and the current position.
     * @param goal The goal position.
     * @param current The current position.
     * @return The minimum error.
     */
    float min_error(float goal, float current);

#if IMU_FB == false
    /**
     * @brief Get the velocity of the gimbal.
     * @return The velocity of the yaw axis, in rad/s.
     */
    [[nodiscard]] float get_yaw_vel() const { return goal_yaw_vel; }

    /**
     * @brief Get the velocity of the gimbal.
     * @return The velocity of the pitch axis, in rad/s.
     */
    [[nodiscard]] float get_pitch_vel() const { return goal_pitch_vel; }
#else // IMU_FB == true

    /**
     * @brief Get the voltage of the gimbal.
     * @return The voltage of the yaw axis, in V.
     */
    [[nodiscard]] float get_yaw_vol() const { return goal_yaw_vol; }

    /**
     * @brief Get the voltage of the gimbal.
     * @return The voltage of the pitch axis, in V.
     */
    [[nodiscard]] float get_pitch_vol() const { return goal_pitch_vol; }
#endif // IMU_FB

private:
    float goal_yaw_vel = 0.0; ///< The goal yaw velocity of the gimbal, in rad/s.
    float goal_pitch_vel = 0.0; ///< The goal pitch velocity of the gimbal, in rad/s.

    float current_yaw_pos = 0.0; ///< The current yaw position of the gimbal, in radians, relative to north.
    float current_pitch_pos = 0.0; ///< The current pitch position of the gimbal, in radians.

    float goal_yaw_pos = 0.0; ///< The goal yaw position of the gimbal, in radians, relative to north.
    float goal_pitch_pos = 0.0; ///< The goal pitch position of the gimbal, in radians.

    float yaw_pos_error; ///< The error of the yaw position of the gimbal, in radians.
    float pitch_pos_error; ///< The error of the pitch position of the gimbal, in radians.

    PidParam yaw_p2v_param;
    PidParam pitch_p2v_param;

    PidOutput yaw_p2v_output;
    PidOutput pitch_p2v_output;

#if IMU_FB == true
    float current_yaw_vel = 0.0;
    float current_pitch_vel = 0.0;

    float yaw_vel_error;
    float pitch_vel_error;

    float goal_yaw_vol = 0.0; ///< The goal yaw voltage of the gimbal, in rad/s.
    float goal_pitch_vol = 0.0; ///< The goal pitch voltage of the gimbal, in rad/s.

    PidParam yaw_v2v_param;
    PidParam pitch_v2v_param;

    PidOutput yaw_v2v_output;
    PidOutput pitch_v2v_output;
#endif // IMU_FB

    float omega = 0.0; ///< The angular velocity of the chassis, in rad/s.
    float ratio = 0.7; ///< Ratio of compensate omega.

    std::thread calc_thread; ///< The thread for PID calculation.

    /**
     * @brief Calculate the velocity of the gimbal.
     */
    void calc_vel();

#if IMU_FB == true
    /**
     * @brief Calculate the voltage of the gimbal.
     */
    void calc_vol();
#endif // IMU_FB
};

#endif // GIMBAL_PID_H