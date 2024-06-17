#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

#include <cmath>
#include <algorithm>

/**
 * @brief Represents the data of a motor.
 */
struct MotorData
{
public:
    double torque;
    double velocity; // rad/s
    double position; // cumulative position, rad

    /**
     * @brief Construct a new MotorData object.
     * Set all the data to zero.
     */
    MotorData()
    {
        torque = 0;
        velocity = 0;
        position = 0;
    }

    /**
     * @brief Update the cumulative position of the motor.
     * @param pos The new feedback position.
     */
    void update_pos(double pos)
    {
        position += min_error(pos, position);
    }

private:
    /**
     * @brief Calculate the minimum error in (-M_PI, M_PI] between two angles.
     * Similar to `a - b`, but in the range (-M_PI, M_PI].
     * @param a The first angle.
     * @param b The second angle.
     * @return The minimum error in (-M_PI, M_PI].
     */
    double min_error(double a, double b)
    {
        double diff = a - b;
        while (diff > M_PI) diff -= 2 * M_PI;
        while (diff < -M_PI) diff += 2 * M_PI;
        return diff;
    }
};

/**
 * @brief Represents the parameters of a PID controller.
 */
struct PidParam
{
public:
    double kp;
    double ki;
    double kd;

    PidParam(double kp, double ki, double kd)
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
    double p;
    double i;
    double d;

    double sum()
    {
        return p + i + d;
    }

    PidOutput() : p(0.0), i(0.0), d(0.0) {}
};

#endif