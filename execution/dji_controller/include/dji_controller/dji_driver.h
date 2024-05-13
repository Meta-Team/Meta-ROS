#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "can_driver.hpp"
#include "dji_controller/motor_data.hpp"
#include "dji_controller/can_port.hpp"
#include <linux/can.h>
#include <memory>
#include <vector>
#include <thread>
#include <unordered_map>

#define CALC_FREQ 1 // ms
#define TX_FREQ 2 // ms

#define CALI_TIMEOUT 5 // seconds
#define JAMMED_THRESHOLD 0.3 // s

#define I_MAX 20 // Ampere, current limit
#define V_MAX 300 // to be tuned, velocity limit

#define NaN std::nan("")

#define ENABLE_TIMEOUT false

using std::vector;
using std::unique_ptr;
using std::string;
using std::unordered_map;

#define umap unordered_map

enum MotorType
{
    M3508 = 0,
    M6020 = 1,
    M2006 = 2,
};

/**
 * @brief The DjiDriver class represents a driver for controlling a DJI motor.
 * 
 * This class provides methods for setting goals, configuring PID parameters, 
 * writing data to the motor, and processing received data.
 */
class DjiDriver
{
public:
    int hid; /**< Hardware ID of the motor. */
    string rid; /**< ROS ID of the motor. */

    /**
     * @brief Constructor for DjiDriver class.
     * @param rid The ROS ID of the motor.
     * @param hid The hardware ID of the motor.
     * @param type The type of the motor.
     * @param port The port name of the CAN bus.
     * @param cali The direction of how to calibrate the motor's zero position.
     */
    DjiDriver(const string& rid, int hid, string type, string port, int cali);

    /**
     * @brief Destructor for DjiDriver class.
     */
    ~DjiDriver();
    
    /**
     * @brief Set the desired position and velocity for the motor.
     * @param goal_pos The desired position.
     * @param goal_vel The desired velocity.
     */
    void set_goal(double goal_pos, double goal_vel, double goal_cur);

    /**
     * @brief Set the PID parameters for position-to-velocity conversion.
     * @param kp The desired proportional gain.
     * @param ki The desired integral gain.
     * @param kd The desired derivative gain.
     */
    void set_p2v_pid(double kp, double ki, double kd);

    /**
     * @brief Set the PID parameters for velocity-to-current conversion.
     * @param kp The desired proportional gain.
     * @param ki The desired integral gain.
     * @param kd The desired derivative gain.
     */
    void set_v2c_pid(double kp, double ki, double kd);

    /**
     * @brief Get the current state of the motor.
     * @return A tuple containing the position, velocity, and current of the motor.
     */
    [[nodiscard]] std::tuple<double, double, double> get_state();

    /**
     * @brief Get the port number of the CAN bus.
     * @return The port number.
     */
    [[nodiscard]] int get_port() const { return port; }

private:
    static umap<int, unique_ptr<CanPort>> can_ports; /**< Array of pointers to the CAN port instances. */
    static umap<int, std::thread> rx_threads; /**< Array of threads for the feedback loop. */
    static umap<int, std::thread> tx_threads; /**< Array of threads for the transmit loop. */
    /**
     * @brief A vector of shared pointers to the DjiDriver instances.
     * Used for feedback loop for processing received data.
     */
    static vector<std::shared_ptr<DjiDriver>> instances;

    uint8_t port; /**< CAN port number. */
    MotorType motor_type; /**< Type of the motor. */
    PidParam p2v_prm, v2c_prm; /**< PID parameters for position-to-velocity and velocity-to-current conversion. */
    PidOutput p2v_out, v2c_out; /**< PID outputs for position-to-velocity and velocity-to-current conversion. */

    MotorData present_data{}; /**< Data representing the current state of the motor. */

    double vel_error{}; /**< Error in velocity. */
    double pos_error{}; /**< Error in position. */
    double goal_pos{}; /**< Desired position of the motor. */
    double goal_vel{}; /**< Desired velocity of the motor. */
    double current{}; /**< Current value of the motor. */
    double zero = 0.0; /**< Zero position of the motor. */

    double last_command; /**< When the motor receives the last command. */
    bool ready; /**< Flag to indicate if the motor is ready to receive commands. */
#if ENABLE_TIMEOUT
    std::thread timeout_thread; /**< Thread for checking timeout. */
#endif
    std::thread calc_thread; /**< Thread for calculating PID control. */
    std::thread cali_thread; /**< Thread for calibrating the motor. */

#if ENABLE_TIMEOUT
    /**
     * @brief Check if the motor is timed out.
     * If the motor is timed out, the current value is set to zero.
     */
    void check_timeout(); /**< Check if the motor is timed out. */
#endif

    /**
     * @brief Convert velocity to current using PID control.
     * This depends on member variables `goal_vel`, `present_data.velocity`.
     * The result is stored in member variable `current`.
     */
    void vel2current();

    /**
     * @brief Convert position to velocity using PID control.
     * This depends on member variables `goal_pos`, `present_data.position`.
     * The result is stored in member variable `goal_vel`.
     * @note To realize position control, this function should be executed before vel2current().
     */
    void pos2velocity();

    /**
     * @brief Set the CAN port number of this motor, and add it to the array of CAN ports.
     * @param port The port number.
     * @note This create a new feedback loop thread if the port is not in the array.
     */
    void set_port(int port);

    /**
     * @brief Receive loop for processing received data.
     * @param port The port number of the CAN bus.
     * @note Each port should have a separate thread for receiving data.
     */
    static void rx_loop(int port);

    /**
     * @brief Transmit loop for sending data to the motor.
     * @param port The port number of the CAN bus.
     * @note Each port should have a separate thread for transmitting data.
     */
    static void tx_loop(int port);

    /**
     * @brief Calibration loop for measuring the zero position of the motor.
     * @param dir The direction of calibration.
     * @note This function is executed once when the motor is initialized.
     */
    void cali_loop(int dir);

    /**
     * @brief Calculate the PID control.
     * This function is executed in a separate thread.
     * @note Each motor should have a separate thread for calculating PID control.
     */
    void calc_loop(); /**< Loop for calculating PID control. */

    /**
     * @brief Calculate and write the transmit frames.
     * PID control is executed in this function.
     * Position-to-velocity conversion is executed iff goal_pos is not zero.
     * @note This function should be executed before sending the transmit frame.
     */
    void calc_tx();

    /**
     * @brief Process the receive frame.
     * This updates the present data of the motor according to the rx_frame variable.
     * @note rx() should be executed before calling this function.
     */
    void process_rx();

    /**
     * @brief Limits the value of a variable to a specified range.
     * @param val Reference to the variable to be limited.
     * @param limit The upper and lower limit for the value.
     * @note Limit must be positive.
     */
    void curb(double &val, double limit);
};

#endif // DJI_DRIVER_H