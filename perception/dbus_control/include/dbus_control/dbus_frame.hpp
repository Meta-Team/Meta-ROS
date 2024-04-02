#ifndef DBUS_FRAME_HPP
#define DBUS_FRAME_HPP

#include <cstdint>
#include <vector>

#include "operation_interface/msg/dbus_control.hpp"

#define CH_MAX 1684
#define CH_MIN 364

class DbusFrame
{
public:
    struct [[gnu::packed]] FrameType
    {
        struct [[gnu::packed]] RemoteControl
        {
            uint16_t ch0: 11; /**< Right stick, right is positive. */
            uint16_t ch1: 11; /**< Right stick, down is positive. */
            uint16_t ch2: 11; /**< Left stick, right is positive. */
            uint16_t ch3: 11; /**< Left stick, down is positive. */
            uint8_t s1: 2; /**< Left switch */
            uint8_t s2: 2; /**< Right switch */
        };

        struct [[gnu::packed]] Mouse
        {
            uint16_t x;
            uint16_t y;
            uint16_t z;
            uint8_t left;
            uint8_t right;
        };

        RemoteControl rc;
        Mouse mouse;
        uint16_t keyboard;
        uint16_t reserved;
    };

    FrameType interpreted; /**< The interpreted frame */

    DbusFrame(const std::vector<uint8_t> &frame);

    operation_interface::msg::DbusControl msg();

private:
    float linear_mapping(int raw, int raw_min, int raw_max, float min, float max);
};

#endif // DBUS_FRAME_HPP