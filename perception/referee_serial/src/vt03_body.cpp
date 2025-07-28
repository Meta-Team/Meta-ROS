#include "referee_serial/vt03_body.hpp"
#include <bitset>
#include <cstdint>
#include <vector>


VT03_MSG::VT03_MSG(const std::vector<uint8_t> &frame)
{
    // copy the uint8_t vector to the struct
    std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t*>(&interpreted));
}

operation_interface::msg::VT03 VT03_MSG::msg()
{
    operation_interface::msg::VT03 msg;
    // mouse
    // TODO casting to double
    msg.ch0 = this->interpreted.ch0;
    msg.ch1 = this->interpreted.ch1;
    msg.ch2 = this->interpreted.ch2;
    msg.ch3 = this->interpreted.ch3;
    msg.cns = this->interpreted.cns;
    msg.pause = this->interpreted.pause;
    msg.left_btn  = this->interpreted.left_btn;
    msg.right_btn = this->interpreted.right_btn;
    msg.wheel = this->interpreted.wheel;
    msg.trigger = this->interpreted.trigger;
    msg.mouse_x = this->interpreted.mouse_x;
    msg.mouse_y = this->interpreted.mouse_y;
    msg.mouse_z = this->interpreted.mouse_z;
    msg.mouse_left = this->interpreted.mouse_left;
    msg.mouse_right = this->interpreted.mouse_right;
    msg.mouse_middle = this->interpreted.mouse_middle;
    msg.w = this->interpreted.w;
    msg.s = this->interpreted.s;
    msg.a = this->interpreted.a;
    msg.d = this->interpreted.d;
    msg.shift = this->interpreted.shift;
    msg.ctrl = this->interpreted.ctrl;
    msg.q = this->interpreted.q;
    msg.e = this->interpreted.e;
    msg.r = this->interpreted.r;
    msg.f = this->interpreted.f;
    msg.g = this->interpreted.g;
    msg.z = this->interpreted.z;
    msg.x = this->interpreted.x;
    msg.c = this->interpreted.c;
    msg.v = this->interpreted.v;
    msg.b = this->interpreted.b;
    // return
    return msg;
}