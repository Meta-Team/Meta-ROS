#include "serial_driver/serial_driver.hpp"
#include <serial_driver/serial_port.hpp>
#include <string>

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::StopBits;
using drivers::serial_driver::SerialPortConfig;