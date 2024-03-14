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
        max_reached = false;
        min_reached = true;
    }

    void operator ++()
    {
        rising = true;
        if (count_down < COUNT) {
            count_down++;
            max_reached = false;
        }
        determine();
    }

    void operator --()
    {
        rising = false;
        if (count_down > 0) {
            count_down--;
            min_reached = false;
        }
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
    bool max_reached;
    bool min_reached;
    rclcpp::Logger logger;

    void determine()
    {
        if (rising && count_down == COUNT && !max_reached)
        {
            active = true;
            max_reached = true;
            RCLCPP_INFO(logger, "Target acquired");
        }
        else if (!rising && count_down == 0 && !min_reached)
        {
            active = false;
            min_reached = true;
            RCLCPP_INFO(logger, "Target lost");
        }
        // else do nothing
    }
};

#endif // AIM_MODE_HPP