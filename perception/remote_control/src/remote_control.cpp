#include "remote_control/remote_control.hpp"
#include "remote_control/remote.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include "remote_control/crc.h"

constexpr const char * RemoteControl::dev_name;
constexpr const char * RemoteControl::dev_null;
constexpr uint32_t RemoteControl::baud;
constexpr FlowControl RemoteControl::fc;
constexpr Parity RemoteControl::pt;
constexpr StopBits RemoteControl::sb;

RemoteControl::RemoteControl(const rclcpp::NodeOptions & options)
{
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    node_ = rclcpp::Node::make_shared("remote_control", options);
    pub_ = node_->create_publisher<operation_interface::msg::RemoteControl>("remote_control", 10);

    // open serial port
    if (!port_->is_open())
    {
        port_->open();
        receive_thread = std::thread(&RemoteControl::receive, this);
    }
    RCLCPP_INFO(node_->get_logger(), "RemoteControl node started");
}

RemoteControl::~RemoteControl()
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

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr RemoteControl::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void RemoteControl::receive()
{
    std::vector<uint8_t> prefix(7); // header + cmd_id
    std::vector<uint8_t> frame;
    frame.reserve(sizeof(Remote::RemoteFrame));
    RCLCPP_INFO(node_->get_logger(), "Receiving remote control frames");

    while (rclcpp::ok())
    {
        try {
            port_->receive(prefix);

            if (Remote::is_wanted_pre(prefix))
            {
                // read the rest of the frame
                frame.resize(sizeof(Remote::RemoteFrame) - prefix.size());
                port_->receive(frame);
                frame.insert(frame.begin(), prefix.begin(), prefix.end());
                 // interpret frame
                Remote remote(frame);

                bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&remote.interpreted), sizeof(Remote::RemoteFrame));

                if (crc16_check)
                {
                    // publish message
                    auto msg = remote.msg();
                    pub_->publish(msg);
                }
                else {
                    RCLCPP_WARN(node_->get_logger(), "CRC16 check failed");
                }
            }
            else {
                RCLCPP_WARN(node_->get_logger(), "Received unwanted frame");
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error receiving remote control frame: %s", e.what());
            reopen_port();
        }
    }
}

void RemoteControl::reopen_port()
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

RCLCPP_COMPONENTS_REGISTER_NODE(RemoteControl)