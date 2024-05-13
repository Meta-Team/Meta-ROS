#include "rclcpp/rclcpp.hpp"
#include "relay_sucker/sucker.h"

#include "device_interface/msg/relay.hpp"
#include "behavior_interface/msg/grasp.hpp"

class RelaySucker : public rclcpp::Node
{
public:
    RelaySucker() : Node("relay_sucker")
    {
        relay_pub_ = this->create_publisher<device_interface::msg::Relay>("relay", 10);
        grasp_sub_ = this->create_subscription<behavior_interface::msg::Grasp>("grasp", 10,
            std::bind(&RelaySucker::grasp_callback, this, std::placeholders::_1));

        sucker_ = std::make_unique<Sucker>(relay_pub_);

        RCLCPP_INFO(this->get_logger(), "RelaySucker initialized");
    }

private:
    rclcpp::Publisher<device_interface::msg::Relay>::SharedPtr relay_pub_;
    rclcpp::Subscription<behavior_interface::msg::Grasp>::SharedPtr grasp_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<Sucker> sucker_;

    void grasp_callback(const behavior_interface::msg::Grasp::SharedPtr msg)
    {
        sucker_->input(msg->enable);
    }

    // publishing is done in the Sucker class
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelaySucker>());
    rclcpp::shutdown();
    return 0;
}