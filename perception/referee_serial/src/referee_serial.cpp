#include "referee_serial/referee_serial.hpp"
#include "referee_serial/key_mouse.hpp"
#include "referee_serial/game_info.hpp"
#include "referee_serial/power_state.hpp"
#include "referee_serial/custom_controller.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>
#include "referee_serial/crc.h"

#define DEBUG false

std::string RefereeSerial::dev_name;
constexpr const char * RefereeSerial::dev_null;
constexpr uint32_t RefereeSerial::baud;
constexpr FlowControl RefereeSerial::fc;
constexpr Parity RefereeSerial::pt;
constexpr StopBits RefereeSerial::sb;

RefereeSerial::RefereeSerial(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("referee_serial", options);

    // create serial port
    dev_name = node_->declare_parameter("serial_path", "/dev/ttyUSB1");
    ctx_ = std::make_unique<IoContext>(2);
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, dev_name, *config_);
    RCLCPP_INFO(node_->get_logger(), "Using serial port: %s", dev_name.c_str());

    // create publishers
    key_mouse_pub_ = node_->create_publisher<operation_interface::msg::KeyMouse>("key_mouse", 10);
    game_info_pub_ = node_->create_publisher<operation_interface::msg::GameInfo>("game_info", 10);
    power_state_pub_ = node_->create_publisher<operation_interface::msg::PowerState>("power_state", 10);
    custom_controller_pub_ = node_->create_publisher<operation_interface::msg::CustomController>("custom_controller", 10);

    // open serial port
    if (!port_->is_open())
    {
        port_->open();
        receive_thread = std::thread(&RefereeSerial::receive, this);
    }

    RCLCPP_INFO(node_->get_logger(), "RefereeSerial initialized");
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
    RCLCPP_INFO(node_->get_logger(), "Receiving serial frames");

    while (rclcpp::ok())
    {
        try {
            port_->receive(prefix);

            if (KeyMouse::is_wanted_pre(prefix)) // key mouse
            {
                handleFrame<operation_interface::msg::KeyMouse, KeyMouse>(
                    prefix, key_mouse_pub_, "key_mouse");
            }
            else if (GameInfo::is_wanted_pre(prefix)) // game info
            {
                handleFrame<operation_interface::msg::GameInfo, GameInfo>(
                    prefix, game_info_pub_, "game_info");
            }
            else if (PowerState::is_wanted_pre(prefix)) // power state
            {
                handleFrame<operation_interface::msg::PowerState, PowerState>(
                    prefix, power_state_pub_, "power_state");
            }
            else if (CustomController::is_wanted_pre(prefix)) // custom controller
            {
                handleFrame<operation_interface::msg::CustomController, CustomController>(
                    prefix, custom_controller_pub_, "custom_controller");
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
            RCLCPP_ERROR(node_->get_logger(), "Error receiving frame: %s", e.what());
            reopen_port();
        }
    }
}

template<typename MSG, typename PARSE>
void RefereeSerial::handleFrame(const std::vector<uint8_t>& prefix,
    typename rclcpp::Publisher<MSG>::SharedPtr pub,
    const std::string frame_type)
{
    std::vector<uint8_t> frame;
    frame.resize(sizeof(typename PARSE::FrameType) - prefix.size());
    port_->receive(frame);
    frame.insert(frame.begin(), prefix.begin(), prefix.end());

    PARSE info(frame); // parse the frame
    bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&info.interpreted), sizeof(typename PARSE::FrameType));

    if (crc16_check)
    {
        MSG msg = info.msg();
        pub->publish(msg);
#if DEBUG == true
        RCLCPP_INFO(node_->get_logger(), "Received %s frame", frame_type.c_str());
#endif
    }
    else {
        RCLCPP_WARN(node_->get_logger(), "%s CRC16 check failed", frame_type.c_str());
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