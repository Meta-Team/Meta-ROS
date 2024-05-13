#include "rclcpp/rclcpp.hpp"
#include "serial_relay/serial_relay.hpp"
#include <vector>

SerialRelay::SerialRelay(const rclcpp::NodeOptions& options)
{
    node_ = std::make_shared<rclcpp::Node>("serial_relay", options);

    int count = node_->declare_parameter("relay.count", 0);
    RCLCPP_INFO(node_->get_logger(), "Relay count: %d", count);
    std::vector<std::string> relay_names;
    relay_names = node_->declare_parameter("relay.names", relay_names);
    std::vector<std::string> relay_ports;
    relay_ports = node_->declare_parameter("relay.ports", relay_ports);
    for (int i = 0; i < count; i++)
    {
        relays[relay_names[i]] = std::make_unique<RelayPort>(relay_ports[i], relay_names[i]);
    }

    relay_sub_ = node_->create_subscription<device_interface::msg::Relay>(
        "relay", 10, std::bind(&SerialRelay::relay_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(), "SerialRelay initialized.");
}

SerialRelay::~SerialRelay()
{
}

auto SerialRelay::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void SerialRelay::relay_callback(const device_interface::msg::Relay::SharedPtr msg)
{
    if (relays.find(msg->relay_id) != relays.end())
    {
        if (msg->enable)
            relays[msg->relay_id]->turn_on();
        else
            relays[msg->relay_id]->turn_off();
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "Relay %s not found", msg->relay_id.c_str());
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SerialRelay)