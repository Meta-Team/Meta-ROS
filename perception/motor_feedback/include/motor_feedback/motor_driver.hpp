#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#include "motor_data.hpp"

class MotorDriver
{
public:
    static can_frame rx_frame;
    static CanDriver* can_0;
    
    virtual MotorData process_rx() = 0;

    float raw2actual(uint16_t raw, float actual_max, uint8_t bits)
    {
        return ((float)(raw - (2 << (bits - 2))) * 2 * actual_max)/(float)(2 << (bits - 1));
    }
    
private:
};

#endif // MOTOR_DRIVER_HPP