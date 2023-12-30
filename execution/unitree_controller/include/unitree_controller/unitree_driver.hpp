#ifndef UNITREE_DRIVER_HPP
#define UNITREE_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"

/**
 * @class UnitreeDriver
 * @brief A class to control and manage Unitree motors.
 *
 * This class provides an interface to control Unitree motors. It uses a SerialPort object to communicate with the motors.
 * It also maintains the current goal command and feedback data for the motor.
 */
class UnitreeDriver
{
public:
    /**
     * @brief Construct a new UnitreeDriver object.
     *
     * This constructor initializes a new UnitreeDriver object.
     */
    UnitreeDriver();

    /**
     * @brief Set the goal for the motor.
     *
     * This function sets the goal position and velocity for the motor.
     *
     * @param pos The goal position for the motor.
     * @param vel The goal velocity for the motor.
     *
     * @warning The symbols in MotorCmd and MotorData differ from their documentation.
     */
    void set_goal(float pos, float vel);

    void send_recv();

private:
    SerialPort serial_port; ///< The serial port used to communicate with the motor.
    MotorCmd goal_cmd; ///< The current goal command for the motor. Its symbols differ from the documentation.
    MotorData feedback_data; ///< The current feedback data for the motor. Its symbols differ from the documentation.
};

#endif // UNITREE_DRIVER_HPP