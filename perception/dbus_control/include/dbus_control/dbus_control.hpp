#ifndef DBUS_CONTROL_HPP
#define DBUS_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"

class DbusControl
{
public:
    DbusControl(const rclcpp::NodeOptions & options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
    rclcpp::Node::SharedPtr node_;

    // std::unique_ptr<IoContext> ctx_;
    // std::unique_ptr<SerialPortConfig> config_;
    // std::unique_ptr<SerialPort> port_;
    // std::vector<uint8_t> send_recv_buff;
    // std::thread receive_thread;
    // static std::string dev_name;
    // static constexpr const char* dev_null = "/dev/null";
    // static constexpr uint32_t baud = 115200;
    // static constexpr FlowControl fc = FlowControl::NONE;
    // static constexpr Parity pt = Parity::NONE;
    // static constexpr StopBits sb = StopBits::ONE;
};

#endif // DBUS_CONTROL_HPP