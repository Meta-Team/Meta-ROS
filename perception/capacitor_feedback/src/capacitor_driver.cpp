#include "capacitor_feedback/capacitor_driver.hpp"

CapacitorDriver::CapacitorDriver() : can_0(std::make_unique<CanDriver>(0))
{
    rx_frame.can_id = 0x211;
    rx_frame.can_dlc = 8;
}

void CapacitorDriver::get_frame()
{
    can_0->get_frame(rx_frame);
}

void CapacitorDriver::process_rx()
{
    input_voltage = (float)(rx_frame.data[0] << 8 | (rx_frame.data[1] & 0xff)) / 100.0f;
    capacitor_voltage = (float)(rx_frame.data[2] << 8 | (rx_frame.data[3] & 0xff)) / 100.0f;
    input_current = (float)(rx_frame.data[4] << 8 | (rx_frame.data[5] & 0xff)) / 100.0f;
    output_power = (float)(rx_frame.data[6] << 8 | (rx_frame.data[7] & 0xff)) / 100.0f;
}