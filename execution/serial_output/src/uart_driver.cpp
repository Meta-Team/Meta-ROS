#include "serial_output/uart_driver.hpp"

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

void UartDriver::send(const std::string & data)
{
    // send serial data
    std::vector<uint8_t> buffer(data.begin(), data.end());
    port_->send(buffer);
};

void UartDriver::read(std::string &data)
{
    // read serial data
    std::vector<uint8_t> buffer;
    port_->receive(buffer);
    data = std::string(buffer.begin(), buffer.end());
};