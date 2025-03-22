#ifndef WFLY_CONTROL_HPP
#define WFLY_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "operation_interface/msg/wfly_control.hpp"
#include <memory>

#include "wfly_control/wfly_sbus.hpp"

#define PUB_RATE 20 // ms

class WflyControl
{
public:
    WflyControl(const rclcpp::NodeOptions & options);
    ~WflyControl();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<operation_interface::msg::WflyControl>::SharedPtr wfly_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<WflySbus> sbus_;

    void timer_callback();
};

#endif // WFLY_CONTROL_HPP