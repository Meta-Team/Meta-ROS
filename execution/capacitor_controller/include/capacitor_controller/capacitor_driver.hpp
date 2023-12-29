#ifndef CAPACITOR_DRIVER_HPP
#define CAPACITOR_DRIVER_HPP

#include "can_driver.hpp"

#include <memory>

class CapacitorDriver
{
private:
    std::unique_ptr<CanDriver> can_0;
    can_frame tx_frame;

public:
    CapacitorDriver();

    void set_power(float input_power);

    void send_frame();
};

#endif // CAPACITOR_DRIVER_HPP