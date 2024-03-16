#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#include <queue>

#define Q_SIZE 256
#define CONTROL_R 10 // ms
#define V_MAX 12 // to be tuned, velocity limit
#define DT CONTROL_R / 1000.0 // s

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
    /**
     * @brief Construct a new Gimbal object.
     * @param yaw The PID parameters for the yaw axis.
     * @param pitch The PID parameters for the pitch axis.
     */
    Gimbal(PidParam yaw, PidParam pitch, float comp);

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
     * @brief Get the feedback from ahrs.
     * @param current_yaw_pos The current yaw position of the gimbal, in radians, relative to north.
     * @param current_pitch_pos The current pitch position of the gimbal, in radians.
     */
    void get_feedback(float current_yaw_pos, float current_pitch_pos);

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

    /**
     * @brief Calculate the velocity of the gimbal.
     * @return [yaw_vel, pitch_vel] The velocity of the yaw and pitch axis, in rad/s.
     */
    std::pair<float, float> calc_vel();

private:
    float yaw_vel = 0.0; ///< The goal yaw velocity of the gimbal, in rad/s.
    float pitch_vel = 0.0; ///< The goal pitch velocity of the gimbal, in rad/s.

    float current_yaw_pos = 0.0; ///< The current yaw position of the gimbal, in radians, relative to north.
    float current_pitch_pos = 0.0; ///< The current pitch position of the gimbal, in radians.

    float goal_yaw_pos = 0.0; ///< The goal yaw position of the gimbal, in radians, relative to north.
    float goal_pitch_pos = 0.0; ///< The goal pitch position of the gimbal, in radians.

    float yaw_error; ///< The error of the yaw position of the gimbal, in radians.
    float pitch_error; ///< The error of the pitch position of the gimbal, in radians.

    PidParam yaw_param;
    PidParam pitch_param;

    PidOutput yaw_output;
    PidOutput pitch_output;

    float omega = 0.0; ///< The angular velocity of the chassis, in rad/s.
    float ratio = 0.7; ///< Ratio of compensate omega.
};

#endif // GIMBAL_PID_H