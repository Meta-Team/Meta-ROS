#include "referee_serial/referee_serial.hpp"
#include "referee_serial/key_mouse.hpp"
#include "referee_serial/game_info.hpp"
#include "referee_serial/power_state.hpp"
#include "referee_serial/custom_controller.hpp"
#include "referee_serial/robot_state.hpp"
#include <chrono>
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <vector>
#include <queue>
#include "referee_serial/crc.h"



constexpr FlowControl RefereeSerial::fc;
constexpr Parity RefereeSerial::pt;
constexpr StopBits RefereeSerial::sb;

RefereeSerial::RefereeSerial(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("referee_serial", options);

    // create serial port
    std::string regular_link_dev_name = node_->declare_parameter("regular_link_port", "/dev/ttyUSB0");
    uint32_t regular_link_baud = node_->declare_parameter("regular_link_baud", 115200);
    std::string video_link_dev_name = node_->declare_parameter("video_link_port", "/dev/ttyUSB1");
    uint32_t video_link_baud = node_->declare_parameter("video_link_baud", 115200);
    bool isDebug = node_->declare_parameter("isDebug", false);

    regular_link_ctx_ = std::make_unique<IoContext>(2);
    video_link_ctx_ = std::make_unique<IoContext>(2);

    auto regular_link_config_ = std::make_unique<SerialPortConfig>(regular_link_baud, fc, pt, sb);
    auto video_link_config_ = std::make_unique<SerialPortConfig>(video_link_baud, fc, pt, sb);

    regular_link_serial_driver_ = std::make_unique<SerialDriver>(*regular_link_ctx_);
    regular_link_serial_driver_->init_port(regular_link_dev_name, *regular_link_config_);
    
    video_link_serial_driver_ = std::make_unique<SerialDriver>(*video_link_ctx_);
    video_link_serial_driver_->init_port(video_link_dev_name, *video_link_config_);

    RCLCPP_INFO(node_->get_logger(), "RegularLink using serial port: %s", regular_link_dev_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "VideoLink using serial port: %s", video_link_dev_name.c_str());

    // create publishers
    key_mouse_pub_ = node_->create_publisher<operation_interface::msg::KeyMouse>("video_link_key_mouse", 10);
    game_info_pub_ = node_->create_publisher<operation_interface::msg::GameInfo>("game_info", 10);
    power_state_pub_ = node_->create_publisher<operation_interface::msg::PowerState>("power_state", 10);
    custom_controller_pub_ = node_->create_publisher<operation_interface::msg::CustomController>("custom_controller", 10);
    robot_state_pub_ = node_->create_publisher<operation_interface::msg::RobotState>("robot_state", 10);

    // open serial port
    if (!regular_link_serial_driver_->port()->is_open())
    {
        regular_link_serial_driver_->port()->open();
        regular_link_receive_thread = std::thread(&RefereeSerial::regular_link_receive, this);
    }
    if (!video_link_serial_driver_->port()->is_open())
    {
        video_link_serial_driver_->port()->open();
        video_link_receive_thread = std::thread(&RefereeSerial::video_link_receive, this);
    }

    RCLCPP_INFO(node_->get_logger(), "RefereeSystem initialized");
}

RefereeSerial::~RefereeSerial()
{
    if (regular_link_receive_thread.joinable())
    {
        regular_link_receive_thread.join();
    }
    if (video_link_receive_thread.joinable())
    {
        video_link_receive_thread.join();
    }
    if (regular_link_serial_driver_->port()->is_open())
    {
        regular_link_serial_driver_->port()->close();
    }
    if (video_link_serial_driver_->port()->is_open())
    {
        video_link_serial_driver_->port()->close();
    }
    if (regular_link_ctx_)
    {
        regular_link_ctx_->waitForExit();
    }
    if (video_link_ctx_)
    {
        video_link_ctx_->waitForExit();
    }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr RefereeSerial::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void RefereeSerial::regular_link_receive()
{
    std::vector<uint8_t> prefix; // header + cmd_id
    std::vector<uint8_t> receive_byte(1);
    RCLCPP_INFO(node_->get_logger(), "Receiving serial frames");

    while (rclcpp::ok())
    {
        try {
            regular_link_serial_driver_->port()->receive(receive_byte);
            prefix.push_back(receive_byte[0]);
            if (prefix.size() < 7)
                continue;
            while (prefix.size() > 7)
                prefix.erase(prefix.begin());

            if (GameInfo::is_wanted_pre(prefix)) // game info
            {
                regular_link_handle_frame<operation_interface::msg::GameInfo, GameInfo>(
                    prefix, game_info_pub_, "game_info");
            }
            else if (PowerState::is_wanted_pre(prefix)) // power state
            {
                regular_link_handle_frame<operation_interface::msg::PowerState, PowerState>(
                    prefix, power_state_pub_, "power_state");
            }
            else if (CustomController::is_wanted_pre(prefix)) // custom controller
            {
                regular_link_handle_frame<operation_interface::msg::CustomController, CustomController>(
                    prefix, custom_controller_pub_, "custom_controller");
            }
            else if (RobotState::is_wanted_pre(prefix)) // robot state
            {
                regular_link_handle_frame<operation_interface::msg::RobotState, RobotState>(
                    prefix, robot_state_pub_, "robot_state");
            }
            else if (isDebug && prefix[0] == 0xA5)
            {
                uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);
                RCLCPP_WARN(node_->get_logger(), "Regular link received unwanted frame with cmd_id: 0x%04X", cmd_id);
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error receiving frame: %s", e.what());
            regular_link_reopen_port();
        }
    }
}

void RefereeSerial::video_link_receive()
{
    std::vector<uint8_t> prefix; // header + cmd_id
    std::vector<uint8_t> receive_byte(1);
    RCLCPP_INFO(node_->get_logger(), "Receiving serial frames");

    while (rclcpp::ok())
    {
        try {
            video_link_serial_driver_->port()->receive(receive_byte);
            prefix.push_back(receive_byte[0]);
            if (prefix.size() < 7)
                continue;
            while (prefix.size() > 7)
                prefix.erase(prefix.begin());
            if (KeyMouse::is_wanted_pre(prefix)) // key mouse
            {
                video_link_handle_frame<operation_interface::msg::KeyMouse, KeyMouse>(
                    prefix, key_mouse_pub_, "key_mouse");
            }
            else if (isDebug && prefix[0] == 0xA5)
            {
                uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);
                RCLCPP_WARN(node_->get_logger(), "Video link received unwanted frame with cmd_id: 0x%04X", cmd_id);
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Error receiving frame: %s", e.what());
            video_link_reopen_port();
        }
    }
}

template<typename MSG, typename PARSE>
void RefereeSerial::regular_link_handle_frame(const std::vector<uint8_t>& prefix,
    typename rclcpp::Publisher<MSG>::SharedPtr pub,
    const std::string frame_type)
{
    std::vector<uint8_t> frame;
    frame.resize(sizeof(typename PARSE::FrameType) - prefix.size());
    regular_link_serial_driver_->port()->receive(frame);
    frame.insert(frame.begin(), prefix.begin(), prefix.end());

    PARSE info(frame); // parse the frame
    bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&info.interpreted), sizeof(typename PARSE::FrameType));

    if (crc16_check) {
        MSG msg = info.msg();
        pub->publish(msg);
        if(isDebug)
            RCLCPP_INFO(node_->get_logger(), "Regular link received %s frame", frame_type.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Regular link %s CRC16 check failed", frame_type.c_str());
    }
}
template<typename MSG, typename PARSE>
void RefereeSerial::video_link_handle_frame(const std::vector<uint8_t>& prefix,
    typename rclcpp::Publisher<MSG>::SharedPtr pub,
    const std::string frame_type)
{
    std::vector<uint8_t> frame;
    frame.resize(sizeof(typename PARSE::FrameType) - prefix.size());
    video_link_serial_driver_->port()->receive(frame);
    frame.insert(frame.begin(), prefix.begin(), prefix.end());

    PARSE info(frame); // parse the frame
    bool crc16_check = crc::verifyCRC16CheckSum(reinterpret_cast<uint8_t*>(&info.interpreted), sizeof(typename PARSE::FrameType));

    if (crc16_check) {
        MSG msg = info.msg();
        pub->publish(msg);
        if(isDebug)
        RCLCPP_INFO(node_->get_logger(), "Received %s frame", frame_type.c_str());
    } else {
        RCLCPP_WARN(node_->get_logger(), "Video link CRC16 %s check failed", frame_type.c_str());
    }
}

void RefereeSerial::regular_link_reopen_port()
{
    RCLCPP_WARN(node_->get_logger(), "Reopening regular link serial port");
    try {
        if (regular_link_serial_driver_->port()->is_open())
        {
            regular_link_serial_driver_->port()->close();
        }
        regular_link_serial_driver_->port()->open();
        RCLCPP_INFO(node_->get_logger(), "Regular link serial port reopened");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error reopening regular link serial port: %s", e.what());
        if (rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            regular_link_reopen_port();
        }
    }
}
void RefereeSerial::video_link_reopen_port()
{
    RCLCPP_WARN(node_->get_logger(), "Reopening video link serial port");
    try {
        if (video_link_serial_driver_->port()->is_open())
        {
            video_link_serial_driver_->port()->close();
        }
        video_link_serial_driver_->port()->open();
        RCLCPP_INFO(node_->get_logger(), "Video link serial port reopened");
    }
    catch (const std::exception & e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Error reopening video link serial port: %s", e.what());
        if (rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::seconds(1));
            video_link_reopen_port();
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(RefereeSerial)