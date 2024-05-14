#include "rclcpp/rclcpp.hpp"
#include "dbus_arm/dbus_interpreter.h"
#include <operation_interface/msg/detail/dbus_control__struct.hpp>

class DbusArm : public rclcpp::Node
{
public:
    DbusArm() : Node("dbus_arm")
    {
        // get param
        double linear_vel = this->declare_parameter("control.end_linear_vel", 1.0);
        double angular_vel = this->declare_parameter("control.end_angular_vel", 1.0);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        RCLCPP_INFO(this->get_logger(), "linear_vel: %f, angular_vel: %f, deadzone: %f",
            linear_vel, angular_vel, deadzone);

        interpreter_ = std::make_unique<DbusInterpreter>(linear_vel, angular_vel, deadzone);

        // pub and sub
        end_vel_pub_ = this->create_publisher<behavior_interface::msg::EndVel>("end_vel", 10);
        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10,
            std::bind(&DbusArm::dbus_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PERIOD), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "DbusArm initialized.");
    }
private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    std::unique_ptr<DbusInterpreter> interpreter_;
    rclcpp::Publisher<behavior_interface::msg::EndVel>::SharedPtr end_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void dbus_callback(const operation_interface::msg::DbusControl msg)
    {
        interpreter_->input(msg);
    }

    void timer_callback()
    {
        end_vel_pub_->publish(*interpreter_->get_end_vel());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<DbusArm>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}