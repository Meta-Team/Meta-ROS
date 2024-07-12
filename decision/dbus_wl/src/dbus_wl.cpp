#include <rclcpp/rclcpp.hpp>
#include "dbus_wl/dbus_interpreter.h"

#define PUB_RATE 15 // ms

class DbusWl : public rclcpp::Node
{
public:
    DbusWl() : Node("dbus_wl")
    {
        // get param
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        double height_sens = this->declare_parameter("control.height_sens", 0.3);
        RCLCPP_INFO(this->get_logger(), "max_vel: %f, max_omega: %f, deadzone: %f",
            max_vel, max_omega, deadzone);

        interpreter_ = std::make_unique<DbusInterpreter>(max_vel, max_omega, deadzone, height_sens);

        // pub and sub
        move_pub_ = this->create_publisher<behavior_interface::msg::Move>("move", 10);
        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10,
            std::bind(&DbusWl::dbus_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "DbusWl initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    std::unique_ptr<DbusInterpreter> interpreter_;
    rclcpp::TimerBase::SharedPtr timer_;

    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        interpreter_->input(msg);
    }

    void timer_callback()
    {
        if (!interpreter_->is_active()) return;
        move_pub_->publish(*interpreter_->get_move());
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}