#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

/**
 * @brief Represents the data of a motor.
 */
struct MotorData
{
public:
    float torque;
    float velocity;
    float position;
};

/**
 * @brief Represents the parameters of a PID controller.
 */
struct PidParam
{
public:
    float kp;
    float ki;
    float kd;

    PidParam(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }
};

/**
 * @brief Represents the output of a PID controller.
 */
struct PidOutput
{
public:
    float p;
    float i;
    float d;

    float sum()
    {
        return p + i + d;
    }

    PidOutput() : p(0.0), i(0.0), d(0.0) {}
};

#endif