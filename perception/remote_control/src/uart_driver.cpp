#include "remote_control/uart_driver.hpp"

constexpr const char * UartDriver::dev_name;
constexpr const char * UartDriver::dev_null;
constexpr uint32_t UartDriver::baud;
constexpr FlowControl UartDriver::fc;
constexpr Parity UartDriver::pt;
constexpr StopBits UartDriver::sb;

UartDriver::UartDriver()
{
    IoContext ctx;
    SerialPortConfig config(baud, fc, pt, sb);
    port_ = std::make_unique<SerialPort>(ctx, dev_name, config);
    port_->open();
}

UartDriver::~UartDriver()
{
    port_->close();
}

void UartDriver::send(const std::vector<uint8_t> &data)
{
    // send serial data
    std::vector<uint8_t> buffer(data.begin(), data.end());
    port_->send(buffer);
};

void UartDriver::read(std::vector<uint8_t> &data)
{
    // read serial data
    port_->receive(data);
};