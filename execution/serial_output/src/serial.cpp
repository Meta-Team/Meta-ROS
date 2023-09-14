#include "serial_output/serial.hpp"

Serial::Serial()
{
    IoContext ctx;
    SerialPortConfig config(baud, fc, pt, sb);
    port_.reset(new SerialPort(ctx, dev_name, config));
    port_->open();
}

Serial::~Serial()
{
    port_->close();
}

void Serial::send(const std::string & data)
{
    // send serial data
    std::vector<uint8_t> buffer(data.begin(), data.end());
    port_->send(buffer);
};