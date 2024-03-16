#ifndef AIM_MODE_HPP
#define AIM_MODE_HPP

#include "rclcpp/rclcpp.hpp"

#define COUNT 10

class AimMode
{
public:
    AimMode(rclcpp::Logger logger) : logger(logger)
    {
        active = false;
        count_down = 0;
        max_reached = false;
        min_reached = true;
    }

    void operator ++()
    {
        count_down = COUNT;
        determine();
    }

    void operator --()
    {
        --count_down;
        if (count_down < 0) count_down = 0;
        determine();
    }

    bool is_active() const
    {
        return active;
    }

private:
    int count_down;
    bool active;
    bool max_reached;
    bool min_reached;
    rclcpp::Logger logger;

    void determine()
    {
        RCLCPP_INFO(logger, "count = %d", count_down);
        if (count_down == COUNT && !max_reached)
        {
            active = true;
            max_reached = true;
            RCLCPP_INFO(logger, "Target acquired");
        }
        else if (count_down == 0 && !min_reached)
        {
            active = false;
            min_reached = true;
            RCLCPP_INFO(logger, "Target lost");
        }
        // else do nothing
    }
};

#endif // AIM_MODE_HPP