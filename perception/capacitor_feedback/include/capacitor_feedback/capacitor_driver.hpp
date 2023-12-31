#ifndef CAPACITOR_DRIVER_HPP
#define CAPACITOR_DRIVER_HPP

#include "can_driver.hpp"

#include <memory>

class CapacitorDriver
{
private:
    std::unique_ptr<CanDriver> can_0;
    can_frame rx_frame;

public:
    CapacitorDriver();

    void get_frame();
    void process_rx();

    float input_voltage{};
    float capacitor_voltage{};
    float input_current{};
    float output_power{};
};

#endif // CAPACITOR_DRIVER_HPP