#include "referee_serial/referee_serial.hpp"
#include "referee_serial/remote_control.hpp"
#include "referee_serial/game_info.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include "referee_serial/crc.h"

#define DEBUG false

constexpr const char * RefereeSerial::dev_name;
constexpr const char * RefereeSerial::dev_null;
constexpr uint32_t RefereeSerial::baud;
constexpr FlowControl RefereeSerial::fc;
constexpr Parity RefereeSerial::pt;
constexpr StopBits RefereeSerial::sb;

RefereeSerial::RefereeSerial(const rclcpp::NodeOptions & options)
{
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    node_ = rclcpp::Node::make_shared("referee_serial", options);
    remote_control_pub_ = node_->create_publisher<operation_interface::msg::RemoteControl>("remote_control", 10);
    game_info_pub_ = node_->create_publisher<operation_interface::msg::GameInfo>("game_info", 10);

    // open serial port
    if (!port_->is_open())
    {
        port_->open();
        receive_thread = std::thread(&RefereeSerial::receive, this);
    }
    RCLCPP_INFO(node_->get_logger(), "RefereeSerial node started");
}

RefereeSerial::~RefereeSerial()
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

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr RefereeSerial::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void RefereeSerial::receive()
{
    std::vector<uint8_t> prefix(7); // header + cmd_id
    std::vector<uint8_t> frame;
    frame.reserve(sizeof(RemoteControl::RemoteFrame));
    RCLCPP_INFO(node_->get_logger(), "Receiving remote control frames");

    while (rclcpp::ok())
    {
        try {
            port_->receive(prefix);

            if (RemoteControl::is_wanted_pre(prefix)) // remote control
            {
                // read the rest of the frame
                frame.resize(sizeof(RemoteControl::RemoteFrame) - prefix.size());
                port_->receive(frame);
                frame.insert(frame.begin(), prefix.begin(), prefix.end());
                 // interpret frame
                RemoteControl rc(frame);

                bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&rc.interpreted), sizeof(RemoteControl::RemoteFrame));

                if (crc16_check)
                {
                    // publish message
                    auto msg = rc.msg();
                    remote_control_pub_->publish(msg);
#if DEBUG == true
                    RCLCPP_INFO(node_->get_logger(), "Received remote control frame");
#endif
                }
                else {
                    RCLCPP_WARN(node_->get_logger(), "RemoteControl CRC16 check failed");
                }
            }
            else if (GameInfo::is_wanted_pre(prefix)) // game info
            {
                // read the rest of the frame
                frame.resize(sizeof(GameInfo::GameInfoFrame) - prefix.size());
                port_->receive(frame);
                frame.insert(frame.begin(), prefix.begin(), prefix.end());
                // interpret frame
                GameInfo gi(frame);

                bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&gi.interpreted), sizeof(GameInfo::GameInfoFrame));

                if (crc16_check)
                {
                    // publish message
                    auto msg = gi.msg();
                    game_info_pub_->publish(msg);
#if DEBUG == true
                    RCLCPP_INFO(node_->get_logger(), "Received game info frame");
#endif
                }
                else {
                    RCLCPP_WARN(node_->get_logger(), "GameInfo CRC16 check failed");
                }
            }
#if DEBUG == true
            else if (prefix[0] == 0xA5)
            {
                uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);
                RCLCPP_WARN(node_->get_logger(), "Received unwanted frame with cmd_id: 0x%04X", cmd_id);
            }
#endif
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error receiving remote control frame: %s", e.what());
            reopen_port();
        }
    }
}

void RefereeSerial::reopen_port()
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

RCLCPP_COMPONENTS_REGISTER_NODE(RefereeSerial)