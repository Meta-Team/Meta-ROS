#include "rclcpp/rclcpp.hpp"
#include "remote_control/remote.hpp"
#include <memory>

#define CONTROL_R 20 // 50 Hz

class RemoteControl : public rclcpp::Node
{
public: 
    RemoteControl() : Node("remote_control")
    {
        remote_ = std::make_unique<Remote>();
        pub_ = this->create_publisher<operation_interface::msg::RemoteControl>("remote_control", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_R), std::bind(&RemoteControl::timer_callback, this));
    }

private:
    std::unique_ptr<Remote> remote_;
    rclcpp::Publisher<operation_interface::msg::RemoteControl>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        remote_->rx_frame();
        remote_->process_rx();
        auto msg = remote_->msg();
        pub_->publish(msg);
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