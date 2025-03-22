#include "rclcpp/rclcpp.hpp"
#include "wfly_control/wfly_control.hpp"

WflyControl::WflyControl(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("dbus_control", options);
    std::string port = node_->declare_parameter("sbus_port", "/dev/wfly_receiver");
    sbus_ = std::make_unique<WflySbus>(port);

    wfly_pub_ = node_->create_publisher<operation_interface::msg::WflyControl>("wfly_control", 10);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(PUB_RATE), std::bind(&WflyControl::timer_callback, this));

    RCLCPP_INFO(node_->get_logger(), "WflyControl initialized");
}

WflyControl::~WflyControl()
{
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr WflyControl::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void WflyControl::timer_callback()
{
    if (!sbus_->valid()) return;
    auto controller_msg = sbus_->controller_msg();
    wfly_pub_->publish(controller_msg);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(WflyControl)