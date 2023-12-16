#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <memory>
#include <thread>

#include "motor_data.hpp"

class MotorDriver
{
public:
    static can_frame rx_frame;
    static std::shared_ptr<CanDriver> can_0;
    
    virtual MotorData process_rx() = 0;

    static void send_frame(const can_frame &tx_frame);
    
private:
    MotorData present_data;
};

#endif // MOTOR_DRIVER_HPP