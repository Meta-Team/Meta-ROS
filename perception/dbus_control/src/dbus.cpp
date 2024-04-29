#include "dbus_control/dbus.h"

#include <cstdint>
#include <fcntl.h>
#include <rclcpp/logging.hpp>
#include <unistd.h>
#include <cstring>

#define termios asmtermios
#include <asm/termios.h>
#undef termios

#include <termios.h>

extern "C" {
extern int ioctl(int __fd, unsigned long int __request, ...) throw();
}

Dbus::Dbus(std::string dev_path)
{
    auto fd = open(dev_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    termios2 options{};
    ioctl(fd, TCGETS2, &options);

    if (fd == -1)
    {
        RCLCPP_ERROR(rclcpp::get_logger("Dbus"), "Failed to open device %s", dev_path.c_str());
        return;
    }

    // Even parity(8E1):
    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;

    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_ispeed = 100000;
    options.c_ospeed = 100000;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~IGNBRK;  // disable break processing

    /* set input mode (non−canonical, no echo,...) */
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    options.c_oflag = 0;                  // no remapping, no delays
    options.c_cflag |= (CLOCAL | CREAD);  // ignore modem controls, enable reading
    ioctl(fd, TCSETS2, &options);

    port = fd;

    // Start read thread
    read_thread = std::thread([this]() {
        while (rclcpp::ok())
        {
            read();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            // sleeping is necessary
        }
    });
}

Dbus::~Dbus()
{
    close(port);
    if (read_thread.joinable()) read_thread.join();
}

operation_interface::msg::DbusControl Dbus::controller_msg()
{
    if (!success) return operation_interface::msg::DbusControl();

    operation_interface::msg::DbusControl msg;

    msg.rs_y = static_cast<double>(-data.ch0) / 660.0; // right stick horizontal, left positive
    msg.rs_x = static_cast<double>(data.ch1) / 660.0; // right stick vertical, up positive
    msg.ls_y = static_cast<double>(-data.ch2) / 660.0; // left stick horizontal, left positive
    msg.ls_x = static_cast<double>(data.ch3) / 660.0; // left stick vertical, up positive
    msg.wheel = static_cast<double>(data.wheel) / 660.0; // wheel
    if (data.s1 != 0) msg.lsw = data.s1;
    if (data.s0 != 0) msg.rsw = data.s0;

    return msg;
}

operation_interface::msg::KeyMouse Dbus::keymouse_msg()
{
    if (!success) return operation_interface::msg::KeyMouse();

    operation_interface::msg::KeyMouse msg;
    // only valid when right switch is at the bottom
    if (data.s0 == 2) msg.active = true;
    else msg.active = false;

    msg.left_button = data.l;
    msg.right_button = data.r;

    msg.mouse_x = static_cast<double>(data.x) / 1600.0;
    msg.mouse_y = static_cast<double>(data.y) / 1600.0;
    msg.mouse_z = static_cast<double>(data.z) / 1600.0;

    msg.w = data.key & 0x01 ? true : false;
    msg.s = data.key & 0x02 ? true : false;
    msg.a = data.key & 0x04 ? true : false;
    msg.d = data.key & 0x08 ? true : false;
    msg.shift = data.key & 0x10 ? true : false;
    msg.ctrl = data.key & 0x20 ? true : false;
    msg.q = data.key & 0x40 ? true : false;
    msg.e = data.key & 0x80 ? true : false;
    msg.r = (data.key >> 8) & 0x01 ? true : false;
    msg.f = (data.key >> 8) & 0x02 ? true : false;
    msg.g = (data.key >> 8) & 0x04 ? true : false;
    msg.z = (data.key >> 8) & 0x08 ? true : false;
    msg.x = (data.key >> 8) & 0x10 ? true : false;
    msg.c = (data.key >> 8) & 0x20 ? true : false;
    msg.v = (data.key >> 8) & 0x40 ? true : false;
    msg.b = (data.key >> 8) & 0x80 ? true : false;

    return msg;
}

void Dbus::unpack()
{
    data.ch0 = (buf[0] | buf[1] << 8) & 0x07FF;
    data.ch0 -= 1024;
    data.ch1 = (buf[1] >> 3 | buf[2] << 5) & 0x07FF;
    data.ch1 -= 1024;
    data.ch2 = (buf[2] >> 6 | buf[3] << 2 | buf[4] << 10) & 0x07FF;
    data.ch2 -= 1024;
    data.ch3 = (buf[4] >> 1 | buf[5] << 7) & 0x07FF;
    data.ch3 -= 1024;
    /* prevent remote control zero deviation */
    if (data.ch0 <= 10 && data.ch0 >= -10)
        data.ch0 = 0;
    if (data.ch1 <= 10 && data.ch1 >= -10)
        data.ch1 = 0;
    if (data.ch2 <= 10 && data.ch2 >= -10)
        data.ch2 = 0;
    if (data.ch3 <= 10 && data.ch3 >= -10)
        data.ch3 = 0;

    data.s0 = ((buf[5] >> 4) & 0x0003);
    data.s1 = ((buf[5] >> 4) & 0x000C) >> 2;

    if ((abs(data.ch0) > 660) || (abs(data.ch1) > 660) || (abs(data.ch2) > 660) ||
        (abs(data.ch3) > 660))
    {
        success = false;
        return;
    }
    data.x = buf[6] | (buf[7] << 8);
    data.y = buf[8] | (buf[9] << 8);
    data.z = buf[10] | (buf[11] << 8);
    data.l = buf[12];
    data.r = buf[13];
    // RCLCPP_INFO(rclcpp::get_logger("Dbus"), "x: %d, y: %d, z: %d, l: %d, r: %d", data.x, data.y, data.z, data.l, data.r);
    data.key = buf[14] | buf[15] << 8;  // key board code
    data.wheel = (buf[16] | buf[17] << 8) - 1024;
    success = true;
}

void Dbus::read()
{
    uint8_t read_byte;
    int timeout = 0;  // time out of one package
    int count = 0;    // count of bit of one package
    while (timeout < 10)
    {
        // Read a byte //
        size_t n = ::read(port, &read_byte, sizeof(read_byte));
        if (n == 0)
        {
            timeout++;
        }
        else if (n == 1)
        {
            // Shift the buffer
            for (int i = 0; i < 17; i++)
            {
                buf[i] = buf[i + 1];
            }
            buf[17] = read_byte;
            count++;
        }
    }
    unpack();
    if (count < 17)
    {
        memset(&data, 0, sizeof(data));
        update = false;
    }
    else {
        update = true;
    }
}