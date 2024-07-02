#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <cstdint>
#include <bitset>
#include <vector>
#include <memory>
#include <sys/types.h>

#include "operation_interface/msg/power_state.hpp"  // CHANGE HERE
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
            uint16_t remain_HP; 
            uint16_t max_HP; 
            uint16_t shooter_id1_17mm_cooling_rate; 
            uint16_t shooter_id1_17mm_cooling_limit; 
            uint16_t shooter_id1_17mm_speed_limit; 
 
            uint16_t shooter_id2_17mm_cooling_rate; 
            uint16_t shooter_id2_17mm_cooling_limit; 
            uint16_t shooter_id2_17mm_speed_limit; 
            uint16_t shooter_id1_42mm_cooling_rate; 
            uint16_t shooter_id1_42mm_cooling_limit; 
            uint16_t shooter_id1_42mm_speed_limit; 
                        
            uint16_t chassis_power_limit; 
            uint8_t mains_power_gimbal_output : 1; 
            uint8_t mains_power_chassis_output : 1; 
            uint8_t mains_power_shooter_output : 1;
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
