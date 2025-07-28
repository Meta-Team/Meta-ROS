#ifndef VT03_BODY_HPP
#define VT03_BODY_HPP
#include "operation_interface/msg/vt03.hpp"
class VT03_MSG
{
public:
    struct __attribute__((packed)) FrameType
    {
        uint16_t frame_header;

        uint16_t ch0:11, ch1:11, ch2:11, ch3:11;
        uint8_t cns:2, pause:1, left_btn:1, right_btn:1;
        uint16_t wheel:11;
        uint8_t trigger:1, dummy_1:3;

        int16_t mouse_x, mouse_y, mouse_z;
        uint8_t mouse_left:2, mouse_right:2, mouse_middle:2, dummy_2:2;

        uint8_t w:1, s:1, a:1, d:1, shift:1, ctrl:1, q:1, e:1, r:1, f:1, g:1, z:1, x:1, c:1, v:1, b:1;

        uint16_t crc16;

    };

    FrameType interpreted; /**< The interpreted frame */ 

    /**
     * @brief Constructs a KeyMouse object from a frame.
     * This would interpret the frame by copying the data into the FrameType struct.
     * @param frame The frame to construct the KeyMouse object from.
     */
    VT03_MSG(const std::vector<uint8_t> &frame);

    /**
     * @brief Sets the message based on the current state.
     * This function copies the current state data into the message object.
     * @note Should be called before publishing the message.
     */
    operation_interface::msg::VT03 msg();
};
#endif