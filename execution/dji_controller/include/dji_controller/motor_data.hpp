#ifndef MOTOR_DATA_HPP
#define MOTOR_DATA_HPP

#include <cmath>
#include <algorithm>

#define PI 3.1415926f

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
    void update_pos(double pos) // MY_TODO: check the first usage of this function
    {
        int round = std::floor(position / 2 / PI);

        double up = 2 * PI * (round + 1) + pos;
        double mid = 2 * PI * round + pos;
        double down = 2 * PI * (round - 1) + pos;

        auto comparison = [this](double a, double b)
        {
            return std::abs(a - position) < std::abs(b - position);
        };

        position = std::min({up, mid, down}, comparison);
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