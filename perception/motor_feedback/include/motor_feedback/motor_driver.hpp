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
    int motor_id;
    MotorData present_data{};
    
    virtual void process_rx() = 0;
    static void get_frame();
    
protected:
    static can_frame rx_frame;
    static std::unique_ptr<CanDriver> can_0;
};

#endif // MOTOR_DRIVER_HPP