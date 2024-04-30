#include "referee_serial/power_state.hpp"
#include <bitset>
#include <cstdint>
#include <vector>

bool PowerState::is_wanted_pre(const std::vector<uint8_t> &prefix)
{
    if (prefix[0] != 0xA5) return false;

    uint16_t length = static_cast<uint16_t>(prefix[1]) | (static_cast<uint16_t>(prefix[2]) << 8);
    uint16_t cmd_id = static_cast<uint16_t>(prefix[5]) | (static_cast<uint16_t>(prefix[6]) << 8);

    if (cmd_id != 0x0202 || length != 16) return false;
    return true;
}

PowerState::PowerState(const std::vector<uint8_t> &frame)
{
    // copy the uint8_t vector to the struct
    std::copy(frame.begin(), frame.end(), reinterpret_cast<uint8_t*>(&interpreted));
}

operation_interface::msg::PowerState PowerState::msg()
{
    operation_interface::msg::PowerState msg;
    msg.chassis_voltage = this->interpreted.data.chassis_voltage;
    msg.chassis_current = this->interpreted.data.chassis_current;
    msg.chassis_power = this->interpreted.data.chassis_power;
    msg.buffer_energy = this->interpreted.data.buffer_energy;
    msg.barrel_17_heat_1 = this->interpreted.data.barrel_17_heat_1;
    msg.barrel_17_heat_2 = this->interpreted.data.barrel_17_heat_2;
    msg.barrel_42_heat = this->interpreted.data.barrel_42_heat;
    return msg;
}