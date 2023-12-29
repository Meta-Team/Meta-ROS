#include "capacitor_controller/capacitor_driver.hpp"

CapacitorDriver::CapacitorDriver()
{
    can_0 = std::make_unique<CanDriver>(0);
    tx_frame.can_id = 0x210;
    tx_frame.can_dlc = 2;
}

void CapacitorDriver::set_power(float input_power)
{
    int16_t power = (uint16_t)(input_power * 100);
    tx_frame.data[0] = (uint8_t)(power >> 8); // high byte
    tx_frame.data[1] = (uint8_t)(power & 0xFF); // low byte
    send_frame();
}

void CapacitorDriver::send_frame()
{
    can_0->send_frame(tx_frame);
}