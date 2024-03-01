#ifndef GIMBAL_PID_H
#define GIMBAL_PID_H

#include <queue>

#define Q_SIZE 256
#define CONTROL_R 10 // ms
#define V_MAX 12 // to be tuned, velocity limit

struct PidParam
{
    float kp = 0.0; ///< The proportional gain of the PID controller.
    float ki = 0.0; ///< The integral gain of the PID controller.
    float kd = 0.0; ///< The derivative gain of the PID controller.

    PidParam(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}
    PidParam() : kp(0), ki(0), kd(0) {}
};

/**
 * @brief A class representing a gimbal.
 */
class Gimbal
{
public:
    Gimbal(PidParam yaw, PidParam pitch);

    void set_goal(float goal_yaw_pos, float goal_pitch_pos);

    void update_omega(float omega);

    void get_feedback(float current_yaw_pos, float current_pitch_pos);

    std::pair<float, float> calc_vel();

private:
    float yaw_vel = 0.0; ///< The goal yaw velocity of the gimbal, in rad/s.
    float pitch_vel = 0.0; ///< The goal pitch velocity of the gimbal, in rad/s.

    float current_yaw_pos = 0.0; ///< The current yaw position of the gimbal, in radians.
    float current_pitch_pos = 0.0; ///< The current pitch position of the gimbal, in radians.

    float goal_yaw_pos = 0.0; ///< The goal yaw position of the gimbal, in radians.
    float goal_pitch_pos = 0.0; ///< The goal pitch position of the gimbal, in radians.

    std::queue<float> yaw_errors; ///< A queue storing errors in yaw.
    std::queue<float> pitch_errors; ///< A queue storing errors in pitch.

    PidParam yaw_param;
    PidParam pitch_param;

    float omega = 0.0; ///< The angular velocity of the chassis, in rad/s.
};

#endif // GIMBAL_PID_H