#include "rclcpp/rclcpp.hpp"
#include "dbus_control/dbus_control.hpp"
#include <string>

DbusControl::DbusControl(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("dbus_control", options);
    std::string port = node_->declare_parameter("dbus_port", "ttyUSB1");
    dbus = std::make_unique<Dbus>("/dev/" + port);

    dbus_pub_ = node_->create_publisher<operation_interface::msg::DbusControl>("dbus_control", 10);
    keymouse_pub_ = node_->create_publisher<operation_interface::msg::RemoteControl>("keymouse_control", 10);

    timer_ = node_->create_wall_timer(std::chrono::milliseconds(PUB_RATE), std::bind(&DbusControl::timer_callback, this));

    RCLCPP_INFO(node_->get_logger(), "DbusControl initialized");
}

DbusControl::~DbusControl()
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr DbusControl::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void DbusControl::timer_callback()
{
    if (!dbus->valid()) return;
    auto controller_msg = dbus->controller_msg();
    auto keymouse_msg = dbus->keymouse_msg();
    dbus_pub_->publish(controller_msg);
    keymouse_pub_->publish(keymouse_msg);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DbusControl)