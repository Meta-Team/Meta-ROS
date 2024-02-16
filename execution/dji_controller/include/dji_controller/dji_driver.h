#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "can_driver.hpp"
#include "dji_controller/motor_data.hpp"
#include <linux/can.h>
#include <memory>
#include <queue>

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f

#define CONTROL_R 1 // ms
#define FEEDBACK_R 1 // ms

#define I_MAX 20 // Ampere, current limit
#define V_MAX 300 // to be tuned, velocity limit

#define Q_SIZE 256 // the size of error queues

enum MotorType
{
    M3508,
    M6020,
    M2006,
};

/**
 * @brief The DjiDriver class represents a driver for controlling a DJI motor.
 * 
 * This class provides methods for setting goals, configuring PID parameters, 
 * writing data to the motor, and processing received data.
 */
class DjiDriver
{
private:
    static std::unique_ptr<CanDriver> can_0; /**< Pointer to the CAN driver instance. */
    static std::unique_ptr<can_frame> tx_frame_200, tx_frame_1ff, tx_frame_2ff; /**< Pointers to CAN frames for transmitting data. */
    static can_frame rx_frame; /**< CAN frame for receiving data. */

    MotorType motor_type; /**< Type of the motor. */
    PidParam p2v_prm, v2c_prm; /**< PID parameters for position-to-velocity and velocity-to-current conversion. */
    PidOutput p2v_out, v2c_out; /**< PID outputs for position-to-velocity and velocity-to-current conversion. */

    MotorData present_data{}; /**< Data representing the current state of the motor. */

    float goal_pos{}; /**< Desired position of the motor. */
    float goal_vel{}; /**< Desired velocity of the motor. */
    std::queue<float> vel_errors; /**< A queue storing errors in vel. */
    std::queue<float> pos_errors; /**< A queue storing errors in pos. */
    float current{}; /**< Current value of the motor. */

    /**
     * @brief Convert velocity to current using PID control.
     * This depends on member variables goal_vel, present_data.velocity.
     * The result is stored in member variable current.
     */
    void vel2current();

    /**
     * @brief Convert position to velocity using PID control.
     * This depends on member variables goal_pos, present_data.position.
     * The result is stored in member variable goal_vel.
     * @note To realize position control, this function should be executed before vel2current().
     */
    void pos2velocity();

    /**
     * @brief Initialize a CAN frame.
     * @param frame_id The ID of the frame.
     * @return A pointer to the CAN frame.
     */
    static std::unique_ptr<can_frame> init_frame(int frame_id);

public:
    int hid; /**< Hardware ID of the motor. */
    int rid; /**< ROS ID of the motor. */

    /**
     * @brief Constructor for DjiDriver class.
     * @param rid The ROS ID of the motor.
     * @param type The type of the motor.
     */
    DjiDriver(int rid, MotorType type);
    
    /**
     * @brief Set the desired position and velocity for the motor.
     * @param goal_pos The desired position.
     * @param goal_vel The desired velocity.
     */
    void set_goal(float goal_pos, float goal_vel);

    /**
     * @brief Set the PID parameters for position-to-velocity conversion.
     * @param kp The desired proportional gain.
     * @param ki The desired integral gain.
     * @param kd The desired derivative gain.
     */
    void set_p2v_pid(float kp, float ki, float kd);

    /**
     * @brief Set the PID parameters for velocity-to-current conversion.
     * @param kp The desired proportional gain.
     * @param ki The desired integral gain.
     * @param kd The desired derivative gain.
     */
    void set_v2c_pid(float kp, float ki, float kd);

    /**
     * @brief Write the transmit frames.
     * PID control is executed in this function.
     * Position-to-velocity conversion is executed iff goal_pos is not zero.
     * @note This function should be executed before sending the transmit frame.
     */
    void write_tx();

    /**
     * @brief Send the transmit frame.
     * This would send all three transmit frames to the motor at once.
     * @note write_tx() should be executed before calling this function.
     */
    static void send_frame();

    /**
     * @brief Get the receive frame.
     * This would write the received frame to the rx_frame variable.
     * @note This function should be executed before calling process_rx().
     */
    static void get_frame();

    /**
     * @brief Process the receive frame.
     * This updates the present data of the motor according to the rx_frame variable.
     * @note get_frame() should be executed before calling this function.
     */
    void process_rx();
};

#endif // DJI_DRIVER_H