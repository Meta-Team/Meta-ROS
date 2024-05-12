#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <unordered_map>
#include "serial_relay/relay_port.h"

#include "behavior_interface/msg/relay.hpp"

class SerialRelay
{
public:
    explicit SerialRelay(const rclcpp::NodeOptions& options);
    ~SerialRelay();
    auto get_node_base_interface() const;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<behavior_interface::msg::Relay>::SharedPtr relay_sub_;
    
    std::unordered_map<std::string, std::unique_ptr<RelayPort>> relays;

    void relay_callback(const behavior_interface::msg::Relay::SharedPtr msg);
};