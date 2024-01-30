#ifndef UART_DRIVER_HPP
#define UART_DRIVER_HPP

// inspired by https://github.com/Meta-Team/Meta-Vision-SolaisNG

#include "serial_driver/serial_driver.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <serial_driver/serial_port.hpp>
#include <string>
#include <sys/socket.h>
#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/remote_control.hpp"

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;

class RemoteControl
{
public:
    RemoteControl(const rclcpp::NodeOptions & options);
    ~RemoteControl();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

    void receive();
    void reopen_port();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<operation_interface::msg::RemoteControl>::SharedPtr pub_;

    std::unique_ptr<IoContext> ctx_;
    std::unique_ptr<SerialPortConfig> config_;
    std::unique_ptr<SerialPort> port_;
    std::vector<uint8_t> send_recv_buff;
    std::thread receive_thread;
    static constexpr const char* dev_name = "/dev/ttyUSB0";
    static constexpr const char* dev_null = "/dev/null";
    static constexpr uint32_t baud = 115200;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;
};

#endif // UART_DRIVER_HPP