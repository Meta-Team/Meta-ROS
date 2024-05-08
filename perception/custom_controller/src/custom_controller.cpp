#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/create_publisher.hpp>
#include "custom_controller/can_interpreter.h"

#include "operation_interface/msg/custom_controller.hpp"

using CC = operation_interface::msg::CustomController;

#define PUB_R 20 // ms

class CustomController : public rclcpp::Node
{
public:
    CustomController() : rclcpp::Node("custom_controller")
    {
        interpreter = std::make_unique<CanInterpreter>();

        cc_pub_ = this->create_publisher<CC>("custom_controller", 10);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_R), [this](){
            this->timer_callback();
        });

        RCLCPP_INFO(this->get_logger(), "CustomController initialized.");
    }

private:
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::Publisher<operation_interface::msg::CustomController>::SharedPtr cc_pub_;

    std::unique_ptr<CanInterpreter> interpreter;

    void timer_callback()
    {
        auto msg = interpreter->get_msg();
        cc_pub_->publish(msg);
    }
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}