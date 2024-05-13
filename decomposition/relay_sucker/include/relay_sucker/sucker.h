#ifndef SUCKER_H
#define SUCKER_H

#include "rclcpp/rclcpp.hpp"
#include "device_interface/msg/relay.hpp"
#include <thread>

#define PUB_RATE 20 // milliseconds
#define DURATION 0.5 // seconds

class Sucker
{
public:
    Sucker(rclcpp::Publisher<device_interface::msg::Relay>::SharedPtr relay_pub);
    ~Sucker();
    
    void input(bool enable);

    device_interface::msg::Relay get_msg();

private:
    std::thread pump_thread_;
    std::thread valve_thread_;
    double last_not_suck;

    device_interface::msg::Relay pump_msg_;
    device_interface::msg::Relay valve_msg_;
    rclcpp::Publisher<device_interface::msg::Relay>::SharedPtr relay_pub_;

    void pump_loop();
    void valve_loop();
};

#endif // SUCKER_H