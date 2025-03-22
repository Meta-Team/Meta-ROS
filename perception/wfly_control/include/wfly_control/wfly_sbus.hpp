#ifndef WFLY_SBUS_HPP
#define WFLY_SBUS_HPP

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <operation_interface/msg/detail/wfly_control__struct.hpp>

#include "operation_interface/msg/wfly_control.hpp"
#include "CSerialPort/SerialPort.h"
#include "CSerialPort/SerialPortInfo.h"

struct WflyData
{
    uint16_t ch1;
    uint16_t ch2;
    uint16_t ch3;
    uint16_t ch4;
    uint16_t ch5;
    uint16_t ch6;
    uint16_t ch7;
    uint16_t ch8;
};

struct [[gnu::packed]] SbusFrame
{
    uint8_t head    : 8;

    // Data packet, 11*16 = 176 bits = 22 bytes
    uint16_t ch1    : 11;
    uint16_t ch2    : 11;
    uint16_t ch3    : 11;
    uint16_t ch4    : 11;
    uint16_t ch5    : 11;
    uint16_t ch6    : 11;
    uint16_t ch7    : 11;
    uint16_t ch8    : 11;
    uint16_t ch9    : 11;
    uint16_t ch10   : 11;
    uint16_t ch11   : 11;
    uint16_t ch12   : 11;
    uint16_t ch13   : 11;
    uint16_t ch14   : 11;
    uint16_t ch15   : 11;
    uint16_t ch16   : 11;

    uint8_t falgs   : 8;
    uint8_t tail    : 8;
};

class WflySbus
{
public:
    WflySbus(std::string dev_path);
    ~WflySbus();

    operation_interface::msg::WflyControl controller_msg();

    bool valid() { return first_packet_arrived_; }

private:
    bool first_packet_arrived_ = false;

    std::string dev_path_;

    void process_packet(const uint8_t* packet_buffer);

    bool read_with_timeout(uint8_t* buffer, size_t bytes_to_read, int timeout_ms);

    operation_interface::msg::WflyControl wfly_msg_;

    // Serial port
    std::unique_ptr<itas109::CSerialPort> serial_port_;

    // RX thread
    std::unique_ptr<std::jthread> rx_thread_;
    void rx_loop(std::stop_token stop_token);
};

#endif // WFLY_SBUS_HPP