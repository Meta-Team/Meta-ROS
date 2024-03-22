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

#include "operation_interface/msg/remote_control.hpp"
#include "operation_interface/msg/game_info.hpp"
#include "operation_interface/msg/power_state.hpp"

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;

class RefereeSerial
{
public:
    RefereeSerial(const rclcpp::NodeOptions & options);
    ~RefereeSerial();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

    void receive();
    void reopen_port();

    template<typename MSG, typename PARSE>
    void handleFrame(const std::vector<uint8_t>& prefix,
        typename rclcpp::Publisher<MSG>::SharedPtr pub,
        const std::string frameType);

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<operation_interface::msg::RemoteControl>::SharedPtr remote_control_pub_;
    rclcpp::Publisher<operation_interface::msg::GameInfo>::SharedPtr game_info_pub_;
    rclcpp::Publisher<operation_interface::msg::PowerState>::SharedPtr power_state_pub_;

    std::unique_ptr<IoContext> ctx_;
    std::unique_ptr<SerialPortConfig> config_;
    std::unique_ptr<SerialPort> port_;
    std::vector<uint8_t> send_recv_buff;
    std::thread receive_thread;
    static std::string dev_name;
    static constexpr const char* dev_null = "/dev/null";
    static constexpr uint32_t baud = 115200;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;
};

#endif // REFEREE_SERIAL_HPP