#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <cstdint>
#include <bitset>
#include <vector>
#include <memory>
#include <sys/types.h>

#include "operation_interface/msg/robot_state.hpp"
#include "referee_serial/referee_serial.hpp"

// refer to ./key_mouse.hpp for explanation

class RobotState
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
            uint8_t robot_id;
            uint8_t robot_level;
            uint16_t current_HP;
            uint16_t maximum_HP;
            uint16_t shooter_barrel_cooling_value;
            uint16_t shooter_barrel_heat_limit;
            uint16_t chassis_power_limit;
            uint8_t power_management_gimbal_output : 1;
            uint8_t power_management_chassis_output : 1;
            uint8_t power_management_shooter_output : 1;
        };

        Header header;
        uint16_t cmd_id;
        Data data;
        uint16_t tail;
    };

    FrameType interpreted;

    static bool is_wanted_pre(const std::vector<uint8_t> &prefix);

    RobotState(const std::vector<uint8_t> &frame);

    operation_interface::msg::RobotState msg();
};

#endif // POWER_STATE_HPP
