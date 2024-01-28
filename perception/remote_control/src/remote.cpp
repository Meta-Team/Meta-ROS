#include "remote_control/remote.hpp"
#include <cstdint>
#include <vector>

std::unique_ptr<UartDriver> Remote::uart_ = std::make_unique<UartDriver>();

void Remote::rx_frame()
{
    uart_->read(frame);
}

void Remote::process_rx()
{
    auto sof = frame[0];
    if (sof != 0xA5) return; // check start of frame

    uint16_t length = static_cast<uint16_t>(frame[1]) | (static_cast<uint16_t>(frame[2]) << 8);
    uint16_t cmd_id = static_cast<uint16_t>(frame[5]) | (static_cast<uint16_t>(frame[6]) << 8);

    if (cmd_id != 0x0304 || length != 0x000C) return; // check command id and length

    constexpr int ofs = 7; // offset to data bytes
    this->mouse_x = static_cast<int16_t>(frame[ofs+0]) | (static_cast<int16_t>(frame[ofs+1]) << 8);
    this->mouse_y = static_cast<int16_t>(frame[ofs+2]) | (static_cast<int16_t>(frame[ofs+3]) << 8);
    this->mouse_z = static_cast<int16_t>(frame[ofs+4]) | (static_cast<int16_t>(frame[ofs+5]) << 8);
    this->left_button = static_cast<int8_t>(frame[ofs+6]);
    this->right_button = static_cast<int8_t>(frame[ofs+7]);
    this->keyboard.key_uint = static_cast<uint16_t>(frame[ofs+8]) | (static_cast<uint16_t>(frame[ofs+9]) << 8);
}

operation_interface::msg::RemoteControl Remote::msg()
{
    operation_interface::msg::RemoteControl msg;
    // mouse
    msg.mouse_x = this->mouse_x;
    msg.mouse_y = this->mouse_y;
    msg.mouse_z = this->mouse_z;
    msg.left_button = (this->left_button == 0) ? false : true;
    msg.right_button = (this->right_button == 0) ? false : true;
    // keyboard
    msg.w = this->keyboard.key_bits[W];
    msg.s = this->keyboard.key_bits[S];
    msg.a = this->keyboard.key_bits[A];
    msg.d = this->keyboard.key_bits[D];
    msg.shift = this->keyboard.key_bits[SHIFT];
    msg.ctrl = this->keyboard.key_bits[CTRL];
    msg.q = this->keyboard.key_bits[Q];
    msg.e = this->keyboard.key_bits[E];
    msg.r = this->keyboard.key_bits[R];
    msg.f = this->keyboard.key_bits[F];
    msg.g = this->keyboard.key_bits[G];
    msg.z = this->keyboard.key_bits[Z];
    msg.x = this->keyboard.key_bits[X];
    msg.c = this->keyboard.key_bits[C];
    msg.v = this->keyboard.key_bits[V];
    msg.b = this->keyboard.key_bits[B];
    // return
    return msg;
}