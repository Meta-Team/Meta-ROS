#ifndef DBUS_CONTROL_HPP
#define DBUS_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "dbus_control/dbus_frame.hpp"

#include "operation_interface/msg/dbus_control.hpp"

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;

class DbusControl
{
public:
    DbusControl(const rclcpp::NodeOptions & options);
    ~DbusControl();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

    void receive();
    void reopen_port();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<operation_interface::msg::DbusControl>::SharedPtr pub_;

    std::unique_ptr<IoContext> ctx_;
    std::unique_ptr<SerialPortConfig> config_;
    std::unique_ptr<SerialPort> port_;
    std::vector<uint8_t> send_recv_buff;
    std::thread receive_thread;
    static std::string dev_name;
    static constexpr const char* dev_null = "/dev/null";
    static constexpr uint32_t baud = 115200;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::EVEN;
    static constexpr StopBits sb = StopBits::ONE;
};

#endif // DBUS_CONTROL_HPP