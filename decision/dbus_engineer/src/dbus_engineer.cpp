#include "rclcpp/rclcpp.hpp"

#include "dbus_engineer/dbus_interpreter.h"

#include "operation_interface/msg/dbus_control.hpp"
#include "behavior_interface/msg/move.hpp"

#define PUB_RATE 20

class DbusEngineer : public rclcpp::Node
{
public:
    DbusEngineer() : Node("dbus_engineer")
    {
        double max_vel = this->declare_parameter("control.tran_vel", 2.0);
        double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        RCLCPP_INFO(get_logger(), "max_vel: %f, aim_sens: %f", max_vel, aim_sens);

        interpreter_ = std::make_unique<DbusInterpreter>(max_vel, aim_sens);
    };

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_control_sub_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::unique_ptr<DbusInterpreter> interpreter_;

    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        interpreter_->dbus_input(msg);
    };

    void pub_callback()
    {
        if (!interpreter_->is_active()) return;
        move_pub_->publish(interpreter_->get_move());
    };
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DbusEngineer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}