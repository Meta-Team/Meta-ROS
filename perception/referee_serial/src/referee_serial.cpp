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
    isDebug = node_->declare_parameter("isDebug", false);
    debugUnhandledShown = node_->declare_parameter("debugUnhandledShown", false);
    regular_link_ctx_ = std::make_unique<IoContext>(2);
    video_link_ctx_ = std::make_unique<IoContext>(2);

    auto regular_link_config_ = std::make_unique<SerialPortConfig>(regular_link_baud, fc, pt, sb);
    auto video_link_config_ = std::make_unique<SerialPortConfig>(video_link_baud, fc, pt, sb);

    regular_link_serial_driver_ = std::make_unique<SerialDriver>(*regular_link_ctx_);
    regular_link_serial_driver_->init_port(regular_link_dev_name, *regular_link_config_);
    
    video_link_serial_driver_ = std::make_unique<SerialDriver>(*video_link_ctx_);
    video_link_serial_driver_->init_port(video_link_dev_name, *video_link_config_);

    RCLCPP_INFO(node_->get_logger(), "Regular link using serial port: %s", regular_link_dev_name.c_str());
    RCLCPP_INFO(node_->get_logger(), "Video link using serial port: %s", video_link_dev_name.c_str());

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
    std::vector<uint8_t> sof(1);
    std::vector<uint8_t> header_left(5-1);
    std::vector<uint8_t> cmd_id(2);
    RCLCPP_INFO(node_->get_logger(), "Regular link receiving serial frames");
    while (rclcpp::ok())
    {
        try {
            switch (regular_link_rxstatus){
                case WAIT_STARTING_BYTE:
                    regular_link_serial_driver_->port()->receive(sof);
                    if(sof[0] == REFEREE_SOF){
                        regular_link_rxstatus = WAIT_REMAINING_HEADER;
                        prefix.insert(prefix.end(), sof.begin(), sof.end());
                    }
                    break;
                case WAIT_REMAINING_HEADER:
                    regular_link_serial_driver_->port()->receive(header_left);
                    prefix.insert(prefix.end(), header_left.begin(), header_left.end());

                    if (crc::verifyCRC8CheckSum(prefix.data(), FRAME_HEADER_LENGTH)) {
                        regular_link_rxstatus = WAIT_CMD_ID_DATA_TAIL;
                    } else {
                        prefix.clear();
                        regular_link_rxstatus = WAIT_STARTING_BYTE;
                    }
                    break;
                case WAIT_CMD_ID_DATA_TAIL:
                    regular_link_serial_driver_->port()->receive(cmd_id);
                    prefix.insert(prefix.end(), cmd_id.begin(), cmd_id.end());

                    if (GameInfo::is_wanted_pre(prefix)) {
                        regular_link_handle_frame<operation_interface::msg::GameInfo, GameInfo>(prefix, game_info_pub_, "game_info");
                    } 
                    else if (PowerState::is_wanted_pre(prefix)) {
                        regular_link_handle_frame<operation_interface::msg::PowerState, PowerState>(prefix, power_state_pub_, "power_state");
                    } else if (CustomController::is_wanted_pre(prefix)) {
                        regular_link_handle_frame<operation_interface::msg::CustomController, CustomController>( prefix, custom_controller_pub_, "custom_controller");
                    } else if (RobotState::is_wanted_pre(prefix)) {
                        regular_link_handle_frame<operation_interface::msg::RobotState, RobotState>(prefix, robot_state_pub_, "robot_state");
                    } 
                    else if (debugUnhandledShown && prefix[0] == REFEREE_SOF) {
                        RCLCPP_WARN(node_->get_logger(), "Regular link received unhandled frame with cmd_id:       0x%.2x%.2x", cmd_id[1], cmd_id[0]);
                    }
                    
                    prefix.clear();
                    regular_link_rxstatus = WAIT_STARTING_BYTE;
                    break;
            }
            
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Regular link error receiving frame: %s", e.what());
            regular_link_reopen_port();
        }
    }
}

void RefereeSerial::video_link_receive()
{
    std::vector<uint8_t> prefix; // header + cmd_id
    std::vector<uint8_t> sof(1);
    std::vector<uint8_t> header_left(5-1);
    std::vector<uint8_t> cmd_id(2);
    RCLCPP_INFO(node_->get_logger(), "Video link receiving serial frames");
    while (rclcpp::ok())
    {
        try {
            switch (video_link_rxstatus){
                case WAIT_STARTING_BYTE:
                    video_link_serial_driver_->port()->receive(sof);
                    if(sof[0] == REFEREE_SOF){
                        video_link_rxstatus = WAIT_REMAINING_HEADER;
                        prefix.insert(prefix.end(), sof.begin(), sof.end());
                    }
                    break;
                case WAIT_REMAINING_HEADER:
                    video_link_serial_driver_->port()->receive(header_left);
                    prefix.insert(prefix.end(), header_left.begin(), header_left.end());

                    if (crc::verifyCRC8CheckSum(prefix.data(), FRAME_HEADER_LENGTH), 0) {
                        video_link_rxstatus = WAIT_CMD_ID_DATA_TAIL;
                    } else {
                        prefix.clear();
                        video_link_rxstatus = WAIT_STARTING_BYTE;
                    }
                    break;
                case WAIT_CMD_ID_DATA_TAIL:
                    video_link_serial_driver_->port()->receive(cmd_id);
                    prefix.insert(prefix.end(), cmd_id.begin(), cmd_id.end());
                    if (KeyMouse::is_wanted_pre(prefix)) // key mouse
                    {
                        video_link_handle_frame<operation_interface::msg::KeyMouse, KeyMouse>(
                            prefix, key_mouse_pub_, "key_mouse");
                    }
                    else if (debugUnhandledShown && prefix[0] == REFEREE_SOF)
                    {
                        RCLCPP_WARN(node_->get_logger(), "Video link received unhandled frame with cmd_id:       0x%.2x%.2x", cmd_id[1], cmd_id[0]);
                    }
                    prefix.clear();
                    video_link_rxstatus = WAIT_STARTING_BYTE;
                    break;
            }
            
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Video link error receiving frame: %s", e.what());
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
    std::vector<uint8_t> compensate_byte(1);
    frame.resize(sizeof(typename PARSE::FrameType) - prefix.size());
    regular_link_serial_driver_->port()->receive(frame);
    frame.insert(frame.begin(), prefix.begin(), prefix.end());
    uint16_t crc16_result;
    
    char* whole_frame_str = new char[200];
    memset(whole_frame_str, 0, 200*sizeof(char));
    for(const auto& iter: frame){
        sprintf(whole_frame_str, "%s %.2x ", whole_frame_str, iter);
    }
     // parse the frame
    if (crc::verifyCRC16CheckSum(frame.data(), frame.size(), &crc16_result)) {
        PARSE info(frame);
        MSG msg = info.msg();
        pub->publish(msg);
        if(isDebug) {
            RCLCPP_INFO(node_->get_logger(), "Regular link received %s frame", frame_type.c_str());
            RCLCPP_INFO(node_->get_logger(), "Regular link %s CRC16 check succeeded: %s with locally crc16:%.4x", frame_type.c_str(), whole_frame_str, crc16_result);
        }
    } else {
        if(frame[sizeof(typename PARSE::FrameType)-1] == 0 && frame[sizeof(typename PARSE::FrameType)-2] == (uint8_t)(crc16_result&0xFF)) {
            bool isSolved = false;
            for(int i{0}; i<=3;i++){ // max 3 trials
                regular_link_serial_driver_->port()->receive(compensate_byte);
                if (compensate_byte[0] == (uint8_t)(crc16_result>>8)&0xFF){
                    PARSE info(frame);
                    MSG msg = info.msg();
                    pub->publish(msg);
                    if (isDebug){
                        RCLCPP_INFO(node_->get_logger(), "Regular link %s CRC16 is matched after compensation", frame_type.c_str());
                    }
                    isSolved = true;
                    break;
                } else if(compensate_byte[0] == REFEREE_SOF){
                    RCLCPP_WARN(node_->get_logger(), "Regular link %s breaks another frame's SOF", frame_type.c_str());
                    break;
                }
            }
            if (!isSolved) {
                RCLCPP_WARN(node_->get_logger(), "Regular link %s CRC16 check failed after compensation trials: %s with locally crc16:%.4x", frame_type.c_str(), whole_frame_str, crc16_result);
            }
        } else {
            RCLCPP_WARN(node_->get_logger(), "Regular link %s CRC16 check failed: %s with locally crc16:%.4x", frame_type.c_str(), whole_frame_str, crc16_result);
        }
        // sometimes the upper 8bit of crc16 on regular link is deferred one byte(with the intermediate byte as 0)
        // [WARN] [1753688311.240984445] [referee_serial]: Regular link game_info CRC16 check failed:  a5  0b  00  04  63  01  00  24  07  00  26  03  f8  4f  98  01  00  00  92  00  with locally crc16:2792
        // [WARN] [1753688311.243475314] [referee_serial]: Later on data: 27  a5  03  00  05  18  04  01  00  00

        // print later on data ONLY FOR DEBUGGING
        // frame.resize(10);
        // memset(whole_frame_str, 0, 200*sizeof(char));
        // regular_link_serial_driver_->port()->receive(frame);
        // for(const auto& iter: frame){
        //     sprintf(whole_frame_str, "%s %.2x ", whole_frame_str, iter);
        // }
        // RCLCPP_WARN(node_->get_logger(), "Later on data:%s", whole_frame_str);
    }
    delete[] whole_frame_str;
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
    uint16_t crc16_result;
    char* whole_frame_str = new char[200];
    memset(whole_frame_str, 0, 200*sizeof(char));
    for(const auto& iter: frame){
        sprintf(whole_frame_str, "%s %.2x ", whole_frame_str, iter);
    }
     // parse the frame
    if (crc::verifyCRC16CheckSum(frame.data(), frame.size(), &crc16_result)) {
        PARSE info(frame);
        MSG msg = info.msg();
        pub->publish(msg);
        if(isDebug) {
            RCLCPP_INFO(node_->get_logger(), "Video link Received %s frame", frame_type.c_str());
            RCLCPP_INFO(node_->get_logger(), "Video link %s CRC16 check succeeded: %s with locally crc16:%.4x", frame_type.c_str(), whole_frame_str, crc16_result);
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Video link %s CRC16 check failed: %s with locally crc16:%.4x", frame_type.c_str(), whole_frame_str, crc16_result);
    }
    delete[] whole_frame_str;
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