#ifndef RELAY_PORT_H
#define RELAY_PORT_H

#include "serial_driver/serial_driver.hpp"

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;

class RelayPort
{
public:
    RelayPort(const std::string& dev_name, const std::string& relay_name);
    ~RelayPort();
    void turn_on();
    void turn_off();
    void toggle();

    std::string name;

private:
    std::unique_ptr<IoContext> ctx_;
    std::unique_ptr<SerialPortConfig> config_;
    std::unique_ptr<SerialPort> port_;
    std::string dev_name; ///< The path to the serial port.
    static constexpr const char* dev_null = "/dev/null";
    static constexpr uint32_t baud = 9600;
    static constexpr FlowControl fc = FlowControl::NONE;
    static constexpr Parity pt = Parity::NONE;
    static constexpr StopBits sb = StopBits::ONE;
};

#endif