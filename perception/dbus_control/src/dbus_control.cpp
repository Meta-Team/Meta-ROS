#include "rclcpp/rclcpp.hpp"
#include "dbus_control/dbus_control.hpp"


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr DbusControl::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}



#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DbusControl)