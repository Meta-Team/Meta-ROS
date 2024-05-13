#include "relay_sucker/sucker.h"

Sucker::Sucker(rclcpp::Publisher<device_interface::msg::Relay>::SharedPtr relay_pub)
    : relay_pub_(relay_pub)
{
    pump_msg_.relay_id = "PUMP";
    pump_msg_.enable = false;
    valve_msg_.relay_id = "VALVE";
    valve_msg_.enable = false;
    last_not_suck = 0.0;
    pump_thread_ = std::thread(&Sucker::pump_loop, this);
    valve_thread_ = std::thread(&Sucker::valve_loop, this);
}

Sucker::~Sucker()
{
    if (pump_thread_.joinable())
        pump_thread_.join();
    if (valve_thread_.joinable())
        valve_thread_.join();
}

void Sucker::input(bool enable)
{
    valve_msg_.enable = enable;
    pump_msg_.enable = enable;
    if (!enable) last_not_suck = rclcpp::Clock().now().seconds();
}

device_interface::msg::Relay Sucker::get_msg()
{
    return pump_msg_;
}

void Sucker::pump_loop()
{
    while (rclcpp::ok())
    {
        // if suck for longer than DURATION, stop sucking
        if (rclcpp::Clock().now().seconds() - last_not_suck > DURATION)
        {
            pump_msg_.enable = false;
        }
        relay_pub_->publish(pump_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(PUB_RATE));
    }
}

void Sucker::valve_loop()
{
    while (rclcpp::ok())
    {
        relay_pub_->publish(valve_msg_);
        rclcpp::sleep_for(std::chrono::milliseconds(PUB_RATE));
    }
}