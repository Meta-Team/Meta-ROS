#ifndef GAME_INFO_HPP
#define GAME_INFO_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdint.h>
#include <bitset>
#include <string>
#include <sys/types.h>
#include <vector>

#include "operation_interface/msg/game_info.hpp"
#include "referee_serial/referee_serial.hpp"

// refer to ./key_mouse.hpp for explanation

class GameInfo
{
private:
    static const std::array<std::string, 6> game_type;

    static const std::array<std::string, 6> game_progress;

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
            uint8_t game_type : 4;
            uint8_t game_progress : 4;
            uint16_t stage_remain_time;
            uint64_t unix_timestamp;
        };

        Header header;
        uint16_t cmd_id;
        Data data;
        uint16_t tail;
    };

    FrameType interpreted;

    static bool is_wanted_pre(const std::vector<uint8_t> &prefix);

    GameInfo(const std::vector<uint8_t> &frame);

    operation_interface::msg::GameInfo msg();
};

#endif  // GAME_INFO_HPP