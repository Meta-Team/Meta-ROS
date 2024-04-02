#include "dbus_control/dbus_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "dbus_control/dbus_control.hpp"
#include <cstdint>
#include <exception>
#include <vector>

std::string DbusControl::dev_name;
constexpr const char * DbusControl::dev_null;
constexpr uint32_t DbusControl::baud;
constexpr FlowControl DbusControl::fc;
constexpr Parity DbusControl::pt;
constexpr StopBits DbusControl::sb;

DbusControl::DbusControl(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("referee_serial", options);

    // create serial port
    dev_name = node_->declare_parameter("serial_path", "/dev/ttyUSB1");
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    RCLCPP_INFO(node_->get_logger(), "Using serial port: %s", dev_name.c_str());
    pub_ = node_->create_publisher<operation_interface::msg::DbusControl>("dbus_control", 10);

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
    while (rclcpp::ok())
    {
        try {
            std::vector<uint8_t> frame(16);
            port_->receive(frame);
            DbusFrame info(frame);
            auto msg = info.msg();
            pub_->publish(msg);
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error receiving remote control frame: %s", e.what());
            reopen_port();
        }
    }
}

void DbusControl::reopen_port()
{
    RCLCPP_WARN(node_->get_logger(), "Reopening serial port");
    try {
        if (port_->is_open())
        {
            port_->close();
        }
        port_->open();
        RCLCPP_INFO(node_->get_logger(), "Serial port reopened");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error reopening serial port: %s", e.what());
        if (rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            reopen_port();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(DbusControl)