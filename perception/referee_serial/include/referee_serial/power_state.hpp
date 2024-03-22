#ifndef POWER_STATE_HPP
#define POWER_STATE_HPP

#include <cstdint>
#include <bitset>
#include <vector>
#include <memory>
#include <sys/types.h>

#include "operation_interface/msg/power_state.hpp"
#include "referee_serial/referee_serial.hpp"

// refer to ./remote_control.hpp for explanation

class PowerState
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
            uint16_t chassis_voltage;
            uint16_t chassis_current;
            float chassis_power;
            uint16_t buffer_energy;
            uint16_t barrel_17_heat_1;
            uint16_t barrel_17_heat_2;
            uint16_t barrel_42_heat;
        };

        Header header;
        uint16_t cmd_id;
        Data data;
        uint16_t tail;
    };

    FrameType interpreted;

    static bool is_wanted_pre(const std::vector<uint8_t> &prefix);

    PowerState(const std::vector<uint8_t> &frame);

    operation_interface::msg::PowerState msg();
};

#endif // POWER_STATE_HPP