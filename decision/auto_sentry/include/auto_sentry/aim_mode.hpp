#ifndef AIM_MODE_HPP
#define AIM_MODE_HPP

#include "rclcpp/rclcpp.hpp"

#define COUNT 10 // 10 * UPDATE_R = 200ms

class AimMode
{
public:
    AimMode(rclcpp::Logger logger) : logger(logger)
    {
        count_down = 0;
        rising = false;
        active = false;
    }

    void operator ++()
    {
        rising = true;
        if (count_down < COUNT) count_down++;
        determine();
    }

    void operator --()
    {
        rising = false;
        if (count_down > 0) count_down--;
        determine();
    }

    bool is_active() const
    {
        return active;
    }

private:
    int count_down;
    bool rising;
    bool active;
    rclcpp::Logger logger;

    void determine()
    {
        if (rising && count_down == COUNT)
        {
            active = true;
            RCLCPP_INFO(logger, "Target acquired");
        }
        else if (!rising && count_down == 0)
        {
            active = false;
            RCLCPP_INFO(logger, "Target lost");
        }
        // else do nothing
    }
};

#endif // AIM_MODE_HPP