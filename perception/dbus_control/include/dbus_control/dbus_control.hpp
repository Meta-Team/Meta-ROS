#ifndef DBUS_CONTROL_HPP
#define DBUS_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "operation_interface/msg/dbus_control.hpp"
#include "operation_interface/msg/key_mouse.hpp"
#include <memory>

#include "dbus.h"

#define PUB_RATE 20 // ms

class DbusControl
{
public:
    DbusControl(const rclcpp::NodeOptions & options);
    ~DbusControl();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
    rclcpp::Node::SharedPtr node_;
    bool enable_key_mouse_;
    rclcpp::Publisher<operation_interface::msg::DbusControl>::SharedPtr dbus_pub_;
    rclcpp::Publisher<operation_interface::msg::KeyMouse>::SharedPtr keymouse_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<Dbus> dbus;

    void timer_callback();
};

#endif // DBUS_CONTROL_HPP