#ifndef UNITREE_DRIVER_HPP
#define UNITREE_DRIVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include "serialPort/SerialPort.h"
#include <thread>
#include <tuple>

#define NaN std::nan("")
#define UPDATE_FREQ 20 // ms

#define CALI_TIMEOUT 5 // seconds
#define TRY_VEL 1.5
#define JAMMED_THRESHOLD 0.3 // s

/**
 * @class UnitreeDriver
 * @brief A class to control and manage Unitree motors.
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
     */
    UnitreeDriver(std::string rid, int hid, std::string port, int cali);

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
     * @note This does NOT call send_recv().
     * @warning The symbols in MotorCmd and MotorData differ from their documentation.
     */
    void set_goal(double goal_pos, double goal_vel);

    /**
     * @brief Change the PID parameters for the motor.
     * @param kp The new p2v_kp value.
     * @param kd The new p2v_kd value.
     * @note This would not call send_recv().
     */
    void set_pid(double kp, double kd);

    /**
     * @brief Send the goal command and receive the feedback data.
     * This would send the goal_cmd and overwrite the feedback_data.
     * @note Used in the control loop.
     */
    void send_recv();

    /**
     * @brief Get the current state of the motor.
     * @return A tuple containing the current position, velocity, and torque of the motor.
     */
    std::tuple<double, double, double> get_state();

private:
    double kp, kd; ///< The PID parameters for the motor.
    SerialPort serial_port; ///< The serial port used to communicate with the motor.
    MotorCmd goal_cmd; ///< The current goal command for the motor. Its symbols differ from the documentation.
    MotorData feedback_data; ///< The current feedback data for the motor. Its symbols differ from the documentation.

    std::thread control_thread; ///< The thread for the control loop.

    /**
     * @brief Stop the motor.
     * This would send a zero command to the motor.
     */
    void stop();

    /**
     * @brief Control loop for the motor.
     * Used in the control_thread.
     * This would send the goal command and receive the feedback data at a fixed rate.
     */
    void control_loop();

    bool ready = false; ///< A flag to indicate if the motor has finished calibration, and is ready to receive commands.
    double zero = 0.0; ///< The zero position of the motor.

    std::thread cali_thread; ///< The thread for the calibration loop.

    /**
     * @brief Calibrate the motor.
     * Rotate the motor and try to find a position where it can be jammed physically.
     * Set that position as the zero position.
     * @note This and control_loop() should not be running at the same time.
     */
    void calibrate(int dir);
};

#endif // UNITREE_DRIVER_HPP