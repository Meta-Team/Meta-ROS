#ifndef REFEREE_SERIAL_HPP
#define REFEREE_SERIAL_HPP

// inspired by https://github.com/Meta-Team/Meta-Vision-SolaisNG

#include "serial_driver/serial_driver.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <serial_driver/serial_port.hpp>
#include <string>
#include <sys/socket.h>
#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/key_mouse.hpp"
#include "operation_interface/msg/game_info.hpp"
#include "operation_interface/msg/power_state.hpp"
#include "operation_interface/msg/custom_controller.hpp"
#include "operation_interface/msg/robot_state.hpp"
#include "operation_interface/msg/vt03.hpp"

#define REFEREE_SOF 0xA5
#define VIDEO_LINK_REMOTECONTROL_LOWER_SOF 0xA9
#define VIDEO_LINK_REMOTECONTROL_UPPER_SOF 0x53

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::SerialDriver;


/**
 * @class RefereeSerial
 * @brief A node for handling serial communication with the referee system.
 * This node should be regestered as a ROS2 component.
 */
class RefereeSerial
{
public:
    enum rx_status_t {
        WAIT_STARTING_BYTE,  // receive bytes one by one, waiting for 0xA5
        WAIT_REMAINING_HEADER,  // receive remaining header after SOF
        WAIT_CMD_ID_DATA_TAIL,  // receive cmd_id, data section and 2-byte CRC16 tailing
        VT03_REMAINING
    };
    enum {
        FRAME_HEADER_LENGTH=5,
        FRAME_VT03_LENGTH=21
    };
/**
     * @brief Constructor for the RefereeSerial class.
     * @param options Node options for the ROS2 node.
     * Initializes the publishers and subscribers for the node.
     * Opens the serial port for communication with the referee system.
     * Starts a thread to receive data from the serial port.
     */
    RefereeSerial(const rclcpp::NodeOptions & options);

    /**
     * @brief Destructor for the RefereeSerial class.
     * Closes the serial port and waits for the receive thread to exit.
     */
    ~RefereeSerial();

    /**
     * @brief Method to get the base interface of the node.
     * @return Shared pointer to the base interface of the node.
     */
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:

    /**
     * @brief Method to receive data from the serial port.
     * Interprets the received data and publishes it to the appropriate topic.
     * @note This would call the handle_frame method, and is called by the receive_thread.
     */
    void regular_link_receive();
    void video_link_receive();

    /**
     * @brief Method to reopen the serial port.
     * Called when the serial port is closed.
     */
    void regular_link_reopen_port();
    void video_link_reopen_port();

    /**
     * @brief Method to handle a frame of data.
     * @tparam MSG The message type to be published.
     * @tparam PARSE The class to parse the data.
     * @param prefix The prefix of the frame.
     * @param pub The publisher for the message.
     * @param frame_type The type of the frame.
     */
    template<typename MSG, typename PARSE>
    void regular_link_handle_frame(const std::vector<uint8_t>& prefix,
        typename rclcpp::Publisher<MSG>::SharedPtr pub,
        const std::string frame_type);
    template<typename MSG, typename PARSE>
    void video_link_handle_frame(const std::vector<uint8_t>& prefix,
        typename rclcpp::Publisher<MSG>::SharedPtr pub,
        const std::string frame_type);

    rclcpp::Node::SharedPtr node_; ///< Pointer to the ROS2 node.
    rclcpp::Publisher<operation_interface::msg::KeyMouse>::SharedPtr key_mouse_pub_;
    rclcpp::Publisher<operation_interface::msg::GameInfo>::SharedPtr game_info_pub_;
    rclcpp::Publisher<operation_interface::msg::PowerState>::SharedPtr power_state_pub_;
    rclcpp::Publisher<operation_interface::msg::CustomController>::SharedPtr custom_controller_pub_;
    rclcpp::Publisher<operation_interface::msg::RobotState>::SharedPtr robot_state_pub_;
    rclcpp::Publisher<operation_interface::msg::VT03>::SharedPtr vt03_msg_pub_;

    // Serial port
    std::unique_ptr<SerialDriver> regular_link_serial_driver_;
    std::unique_ptr<SerialDriver> video_link_serial_driver_;
    
    std::unique_ptr<IoContext> regular_link_ctx_;
    std::unique_ptr<IoContext> video_link_ctx_;

    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;

    std::thread regular_link_receive_thread;
    std::thread video_link_receive_thread;
    
    rx_status_t regular_link_rxstatus = WAIT_STARTING_BYTE, video_link_rxstatus = WAIT_STARTING_BYTE;

    bool isDebug = false;
    bool debugUnhandledShown = false;
    bool warningShown = false;
};

#endif // REFEREE_SERIAL_HPP