#include "rclcpp/rclcpp.hpp"

#include "dbus_engineer/dbus_interpreter.h"

#include "operation_interface/msg/dbus_control.hpp"
#include "behavior_interface/msg/grasp.hpp"
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

        move_pub_ = this->create_publisher<behavior_interface::msg::Move>("move", 10);
        grasp_pub_ = this->create_publisher<behavior_interface::msg::Grasp>("grasp", 10);
        dbus_control_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10, std::bind(&DbusEngineer::dbus_callback, this, std::placeholders::_1));
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&DbusEngineer::pub_callback, this));

        RCLCPP_INFO(get_logger(), "DbusEngineer initialized");
    };

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_control_sub_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    rclcpp::Publisher<behavior_interface::msg::Grasp>::SharedPtr grasp_pub_;
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
        grasp_pub_->publish(interpreter_->get_grasp());
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