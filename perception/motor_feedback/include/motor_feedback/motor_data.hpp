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
    float torque;
    float velocity;
    float position; // cumulative position

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
     * 
     * @param pos The new feedback position.
     */
    void update_pos(float pos)
    {
        int round = std::floor(position / 2 / PI);

        float up = 2 * PI * (round + 1) + pos;
        float mid = 2 * PI * round + pos;
        float down = 2 * PI * (round - 1) + pos;

        auto comparison = [this](float a, float b)
        {
            return std::abs(a - position) < std::abs(b - position);
        };

        position = std::min({up, mid, down}, comparison);
    }
};

#endif // MOTOR_DATA_HPP