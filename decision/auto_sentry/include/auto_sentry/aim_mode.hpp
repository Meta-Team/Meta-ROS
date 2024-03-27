#ifndef AIM_MODE_HPP
#define AIM_MODE_HPP

#include "rclcpp/rclcpp.hpp"

#define MODE_DEBUG false

class AimMode
{
public:
    AimMode(rclcpp::Logger logger, int delay) : reset_delay(delay), logger(logger)
    {
        active = false;
        count_down = 0;
        max_reached = false;
        min_reached = true;
    }

    void operator ++()
    {
        count_down = reset_delay;
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
    int reset_delay;
    int count_down;
    bool active;
    bool max_reached;
    bool min_reached;
    rclcpp::Logger logger;

    void determine()
    {
#if MODE_DEBUG == true
        RCLCPP_INFO(logger, "count = %d", count_down);
#endif // MODE_DEBUG
        if (count_down == reset_delay) active = true;
        if (count_down == 0) active = false;

        if (count_down == reset_delay && !max_reached)
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