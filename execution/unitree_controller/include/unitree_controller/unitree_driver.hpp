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
    int hid; ///< The hardware ID of the motor.
    std::string rid; ///< The ROS ID of the motor.

    /**
     * @brief Construct a new UnitreeDriver object.
     * This constructor initializes a new UnitreeDriver object.
     */
    UnitreeDriver(std::string rid, int hid);

    /**
     * @brief Destroy the UnitreeDriver object.
     * This would send a zero command to the motor.
     */
    ~UnitreeDriver();

    /**
     * @brief Set the goal for the motor.
     * This function sets the goal position and velocity for the motor.
     * @param pos The goal position for the motor.
     * @param vel The goal velocity for the motor.
     * @note This would call send_recv().
     * @warning The symbols in MotorCmd and MotorData differ from their documentation.
     */
    void set_goal(float goal_pos, float goal_vel);

    /**
     * @brief Change the PID parameters for the motor.
     * @param kp The new p2v_kp value.
     * @param kd The new p2v_kd value.
     * @note This would not call send_recv().
     */
    void set_pid(float kp, float kd);

    /**
     * @brief Send the goal command and receive the feedback data.
     * This would send the goal_cmd and overwrite the feedback_data.
     */
    void send_recv();

private:
    float kp, kd; ///< The PID parameters for the motor.
    SerialPort serial_port; ///< The serial port used to communicate with the motor.
    MotorCmd goal_cmd; ///< The current goal command for the motor. Its symbols differ from the documentation.
    MotorData feedback_data; ///< The current feedback data for the motor. Its symbols differ from the documentation.
};

#endif // UNITREE_DRIVER_HPP