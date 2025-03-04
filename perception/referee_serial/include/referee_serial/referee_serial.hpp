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

using spb = asio::serial_port_base;
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
    void receive();

    /**
     * @brief Method to reopen the serial port.
     * Called when the serial port is closed.
     */
    void reopen_port();

    /**
     * @brief Method to handle a frame of data.
     * @tparam MSG The message type to be published.
     * @tparam PARSE The class to parse the data.
     * @param prefix The prefix of the frame.
     * @param pub The publisher for the message.
     * @param frame_type The type of the frame.
     */
    template<typename MSG, typename PARSE>
    void handle_frame(const std::vector<uint8_t>& prefix,
        typename rclcpp::Publisher<MSG>::SharedPtr pub,
        const std::string frame_type);

    rclcpp::Node::SharedPtr node_; ///< Pointer to the ROS2 node.
    rclcpp::Publisher<operation_interface::msg::KeyMouse>::SharedPtr key_mouse_pub_;
    rclcpp::Publisher<operation_interface::msg::GameInfo>::SharedPtr game_info_pub_;
    rclcpp::Publisher<operation_interface::msg::PowerState>::SharedPtr power_state_pub_;
    rclcpp::Publisher<operation_interface::msg::CustomController>::SharedPtr custom_controller_pub_;
    rclcpp::Publisher<operation_interface::msg::RobotState>::SharedPtr robot_state_pub_;

    // Serial port
    std::unique_ptr<IoContext> ctx_;
    std::unique_ptr<SerialPortConfig> config_;
    std::unique_ptr<SerialDriver> serial_driver_;
    static std::string dev_name; ///< The path to the serial port.
    static constexpr const char* dev_null = "/dev/null";
    static constexpr uint32_t baud = 115200;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;

    std::thread receive_thread; ///< Thread to receive data from the serial port.
};

#endif // REFEREE_SERIAL_HPP