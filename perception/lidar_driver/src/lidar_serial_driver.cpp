#include "lidar_driver/lidar_serial_driver.h"
#include <cstdint>

LidarSerialDriver::LidarSerialDriver(int port)
{
    IoContext ctx;
    SerialPortConfig config(baud, fc, pt, sb);
    auto dev_name = std::string("/dev/ttyUSB") + std::to_string(port);
    port_.reset(new SerialPort(ctx, dev_name, config));
    port_->open();
}

LidarSerialDriver::~LidarSerialDriver()
{
    port_->close();
}

void LidarSerialDriver::receive(float & data)
{
    std::vector<uint8_t> buffer;
    port_->receive(buffer);

    // buffer[2] = distance low 8 bit, buffer[3] = distance high 8 bit
    uint16_t data_raw = (buffer[2] & 0xff) | ((buffer[3] & 0xff) << 8);
    data = (float)data_raw;
};