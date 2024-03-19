#include "referee_serial/game_info.hpp"
#include <bitset>
#include <cstdint>
#include <vector>

const std::array<std::string, 6> GameInfo::game_type = 
{
    NULL,
    "RoboMaster",
    "Single",
    "AI",
    "League1v1",
    "League2v2",
};

const std::array<std::string, 6> GameInfo::game_progress = 
{
    "NotStarted",
    "Preparation",
    "SelfCheck",
    "5sCountdown",
    "Fighting",
    "End",
};

bool GameInfo::is_wanted_pre(const std::vector<uint8_t> &prefix)
{
    if (prefix[0] != 0xA5) return false;

    uint16_t length = static_cast<uint16_t>(prefix[1]) | (static_cast<uint16_t>(prefix[2]) << 8);
    uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);

    if (cmd_id != 0x0001 || length != 0x000B) return false;
    return true;
}

GameInfo::GameInfo(const std::vector<uint8_t> &frame)
{
    // copy the uint8_t vector to the struct
    std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t *>(&interpreted));
}

operation_interface::msg::GameInfo GameInfo::msg()
{
    operation_interface::msg::GameInfo msg;
    msg.game_type = game_type[this->interpreted.data.game_type];
    msg.game_progress = game_progress[this->interpreted.data.game_progress];
    msg.remain_time = this->interpreted.data.stage_remain_time;
    msg.time_stamp = this->interpreted.data.unix_timestamp;
    return msg;
}