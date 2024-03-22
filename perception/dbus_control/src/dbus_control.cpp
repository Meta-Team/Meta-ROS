#include "rclcpp/rclcpp.hpp"
#include "dbus_control/dbus_control.hpp"

DbusControl::DbusControl(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("referee_serial", options);

    // create serial port
    dev_name = node_->declare_parameter("serial_path", "/dev/ttyUSB0");
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    RCLCPP_INFO(node_->get_logger(), "Using serial port: %s", dev_name.c_str());

    // open serial port
    if (!port_->is_open())
    {
        port_->open();
        receive_thread = std::thread(&DbusControl::receive, this);
    }

    RCLCPP_INFO(node_->get_logger(), "DbusControl initialized");
}

DbusControl::~DbusControl()
{
    if (receive_thread.joinable())
    {
        receive_thread.join();
    }
    if (port_->is_open())
    {
        port_->close();
    }
    if (ctx_)
    {
        ctx_->waitForExit();
    }
}


rclcpp::node_interfaces::NodeBaseInterface::SharedPtr DbusControl::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void DbusControl::receive()
{
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DbusControl)