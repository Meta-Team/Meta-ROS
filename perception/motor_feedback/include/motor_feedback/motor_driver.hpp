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

    static void update_rx(int &id)
    {
        can_0->get_frame(rx_frame);
        id = rx_frame.can_id;
    }

    virtual MotorData process_rx();
};

#endif // MOTOR_DRIVER_HPP