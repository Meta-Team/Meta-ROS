#include "referee_serial/remote_control.hpp"
#include <bitset>
#include <cstdint>
#include <vector>

bool RemoteControl::is_wanted_pre(const std::vector<uint8_t> &prefix)
{
    if (prefix[0] != 0xA5) return false;

    uint16_t length = static_cast<uint16_t>(prefix[1]) | (static_cast<uint16_t>(prefix[2]) << 8);
    uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);

    if (cmd_id != 0x0304 || length != 0x000C) return false;
    return true;
}

RemoteControl::RemoteControl(const std::vector<uint8_t> &frame)
{
    // copy the uint8_t vector to the struct
    std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t *>(&interpreted));
}

operation_interface::msg::RemoteControl RemoteControl::msg()
{
    operation_interface::msg::RemoteControl msg;
    // mouse
    msg.mouse_x = this->interpreted.data.mouse_x;
    msg.mouse_y = this->interpreted.data.mouse_y;
    msg.mouse_z = this->interpreted.data.mouse_z;
    msg.left_button = (this->interpreted.data.left_button == 0) ? false : true;
    msg.right_button = (this->interpreted.data.right_button == 0) ? false : true;
    // keyboard
    std::bitset<16> keyboard_bits(this->interpreted.data.keyboard);
    msg.w = keyboard_bits[W];
    msg.s = keyboard_bits[S];
    msg.a = keyboard_bits[A];
    msg.d = keyboard_bits[D];
    msg.shift = keyboard_bits[SHIFT];
    msg.ctrl = keyboard_bits[CTRL];
    msg.q = keyboard_bits[Q];
    msg.e = keyboard_bits[E];
    msg.r = keyboard_bits[R];
    msg.f = keyboard_bits[F];
    msg.g = keyboard_bits[G];
    msg.z = keyboard_bits[Z];
    msg.x = keyboard_bits[X];
    msg.c = keyboard_bits[C];
    msg.v = keyboard_bits[V];
    msg.b = keyboard_bits[B];
    // return
    return msg;
}