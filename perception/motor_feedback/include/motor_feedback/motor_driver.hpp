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

/**
 * @brief The MotorDriver class represents a DJI or DM motor driver.
 * 
 * This class provides an interface for controlling and interacting with a motor driver.
 * It contains member functions for processing received data and retrieving frames.
 * The class also includes a motor ID and present data for the motor.
 */
class MotorDriver
{
public:
    int motor_id; /**< The ID of the motor. */
    MotorData present_data{}; /**< The present data of the motor. */
    
    /**
     * @brief Process the received frame and update present_data.
     * 
     * This pure virtual function is responsible for processing the received data from the motor driver.
     */
    virtual void process_rx() = 0;
    
    /**
     * @brief Get the frame.
     * 
     * This static function is used to retrieve a frame.
     */
    static void get_frame()
    {
        can_0->get_frame(rx_frame);
    }
    
protected:
    static can_frame rx_frame; /**< The received frame. */
    static std::unique_ptr<CanDriver> can_0; /**< The CAN driver. */
};

#endif // MOTOR_DRIVER_HPP