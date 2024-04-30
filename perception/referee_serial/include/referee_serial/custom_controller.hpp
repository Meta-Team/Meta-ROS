#ifndef CUSTOM_CONTROLLER_HPP
#define CUSTOM_CONTROLLER_HPP

#include <array>
#include <cstdint>
#include <memory>
#include <stdint.h>
#include <bitset>
#include <sys/types.h>
#include <vector>

#include "operation_interface/msg/custom_controller.hpp"
#include "referee_serial/referee_serial.hpp"

class CustomController
{
public:
    struct [[gnu::packed]] FrameType
    {
        struct [[gnu::packed]] Header
        {
            uint8_t sof = 0xA5;
            uint16_t length;
            uint8_t seq;
            uint8_t crc8;
        };

        struct [[gnu::packed]] Data
        {
            float x;
            float y;
            float z;
            float yaw;
            float pitch;
            float roll;
            bool bt1;
            bool bt2;
            bool bt3;
            bool bt4;
        };

        Header header;
        uint16_t cmd_id;
        Data data;
        uint16_t tail;
    };

    FrameType interpreted;

    static bool is_wanted_pre(const std::vector<uint8_t> &prefix);

    CustomController(const std::vector<uint8_t> &frame);

    operation_interface::msg::CustomController msg();
};

#endif // CUSTOM_CONTROLLER_HPP