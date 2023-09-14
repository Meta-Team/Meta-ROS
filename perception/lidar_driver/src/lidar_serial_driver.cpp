#include "lidar_driver/lidar_serial_driver.h"

LidarSerialDriver::LidarSerialDriver()
{
    IoContext ctx;
    SerialPortConfig config(baud, fc, pt, sb);
    port_.reset(new SerialPort(ctx, dev_name, config));
    port_->open();
}

LidarSerialDriver::~LidarSerialDriver()
{
    port_->close();
}

// void LidarSerialDriver::send(const std::string & data)
// {
//     asio::write(*port_, asio::buffer(data));
// };