#include "serial_relay/relay_port.h"
#include "rclcpp/rclcpp.hpp"
#include <cstdint>

#define DEBUG false

RelayPort::RelayPort(const std::string& dev_name, const std::string& relay_name)
    : dev_name("/dev/" + dev_name)
{
    name = relay_name;
    ctx_ = std::make_unique<IoContext>();
    config_ = std::make_unique<SerialPortConfig>(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(*ctx_, this->dev_name, *config_);
    if (!port_->is_open()) port_->open();
    RCLCPP_INFO(rclcpp::get_logger("relay_serial"), "Relay %s using serial: %s", name.c_str(), dev_name.c_str());
}

RelayPort::~RelayPort()
{
    if (port_->is_open())
        port_->close();
}

void RelayPort::turn_on()
{
    std::vector<uint8_t> command = {0xA0, 0x01, 0x01, 0xA2};
#if DEBUG
    RCLCPP_INFO(rclcpp::get_logger("relay_serial"), "Relay %s ON", name.c_str());
#endif
    port_->send(command);
}

void RelayPort::turn_off()
{
    std::vector<uint8_t> command = {0xA0, 0x01, 0x00, 0xA1};
#if DEBUG
    RCLCPP_INFO(rclcpp::get_logger("relay_serial"), "Relay %s OFF", name.c_str());
#endif
    port_->send(command);
}

void RelayPort::toggle()
{
    std::vector<uint8_t> command = {0xA0, 0x01, 0x04, 0xA5};
#if DEBUG
    RCLCPP_INFO(rclcpp::get_logger("relay_serial"), "Relay %s TOGGLE", name.c_str());
#endif
    port_->send(command);
}