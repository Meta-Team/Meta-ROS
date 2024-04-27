// inspired by https://github.com/rm-controls/rm_control/blob/master/rm_dbus/include/rm_dbus/dbus.h

#ifndef DBUS_H
#define DBUS_H

#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cstdint>

#include "operation_interface/msg/dbus_control.hpp"
#include "operation_interface/msg/remote_control.hpp"

struct DbusData
{
    // controller
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    // mouse
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t l;
    uint8_t r;
    
    // keyboard
    uint16_t key;
};

class Dbus
{
public:
    Dbus(std::string dev_path);
    ~Dbus();

    operation_interface::msg::DbusControl controller_msg();
    operation_interface::msg::RemoteControl keymouse_msg();

    bool valid() { return success; }

private:
    int port;
    DbusData data{};
    std::array<int16_t, 18> buf{};
    bool success = false;
    bool update = false;

    std::atomic<bool> running;
    std::thread read_thread;

    void unpack();

    void read();
};

#endif // DBUS_H