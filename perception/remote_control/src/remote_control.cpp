#include "rclcpp/rclcpp.hpp"

class RemoteControl : public rclcpp::Node
{
public: 
    RemoteControl() : Node("remote_control")
    {
        RCLCPP_INFO(this->get_logger(), "Hello, world!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}