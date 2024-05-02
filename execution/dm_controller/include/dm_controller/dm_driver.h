#ifndef DM_DRIVER_H
#define DM_DRIVER_H

#include "can_driver.hpp"
#include "rclcpp/rclcpp.hpp"

#include "cstdint"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#define START_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}
#define STOP_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}
#define SAVE_ZERO_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe}
#define CLEAR_ERROR_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

// can be read from the debugging assistant
#define P_MAX 3.141593
#define V_MAX 30.0
#define T_MAX 10.0

#define KP_MIN 0.0
#define KP_MAX 500.0
#define KD_MIN 0.0
#define KD_MAX 5.0

#define umap std::unordered_map

/**
 * @brief The DmDriver class represents a driver for a DM motor.
 * The DmDriver class is an abstract base class, which means it cannot be instantiated directly.
 * Derived classes must implement the pure virtual functions set_mode(), set_velocity(), and set_position().
 */
class DmDriver
{
protected:
    static umap<int, std::unique_ptr<CanDriver>> can_drivers; /**< Array of pointers to the CAN port instances. */
    static umap<int, can_frame> rx_frames; /**< Array of receiving frames. */
    static umap<int, std::thread> rx_threads; /**< Array of threads for the feedback loop. */
    /**
     * @brief A vector of shared pointers to the DjiDriver instances.
     * Used for feedback loop for processing received data.
     */
    static std::vector<std::shared_ptr<DmDriver>> instances;

    can_frame tx_frame; ///< The CAN frame used for transmitting data.
    float position; ///< The current position of the motor in rad.
    float velocity; ///< The current velocity of the motor in rad/s.
    float torque; ///< The current torque of the motor.
    int port; ///< The port number of the CAN driver.

    float last_command; ///< When the motor receives the last command.
    std::thread timeout_thread; ///< The thread for the timeout.

public:
    int hid; ///< Hardware ID of the motor controlled by the driver.
    std::string rid; ///< ROS ID of the motor controlled by the driver.

    /**
     * @brief Constructor for the DmDriver class.
     * Start the timeout thread.
     */
    DmDriver(std::string port);

    /**
     * @brief Destructor for the DmDriver class.
     * This destructor turns the motor off.
     */
    virtual ~DmDriver();

    /**
     * @brief Turns on the motor.
     */
    void turn_on();

    /**
     * @brief Turns off the motor.
     */
    void turn_off();

    /**
     * @brief Sets the mode of the motor.
     * This function must be implemented by derived classes.
     */
    virtual void set_mode() = 0;

    /**
     * @brief Sets the velocity of the motor.
     * This function must be implemented by derived classes.
     * @param goal_vel The desired velocity of the motor.
     */
    virtual void set_velocity(float goal_vel) = 0;

    /**
     * @brief Sets the position of the motor.
     * This function must be implemented by derived classes.
     * @param goal_pos The desired position of the motor.
     */
    virtual void set_position(float goal_pos) = 0;

    /**
     * @brief Transmits the frame.
     * This would send the tx_frame to the motor.
     */
    void tx() const;

    /**
     * @brief Processes the received frame and updates the present data.
     */
    void process_rx();

    /**
     * @brief Checks if the motor has timed out.
     * @note This function is called by the timeout_thread.
     */
    void check_timeout();

    /**
     * @brief Gets the state of the motor.
     * @return A tuple containing the position, velocity, and torque of the motor.
     */
    [[nodiscard]] std::tuple<float, float, float> get_state() const;

    /**
     * @brief Converts an integer value to a floating-point value.
     * This static function performs data conversion provided by the DM motor.
     * @param x_int The integer value to be converted.
     * @param x_min The minimum value of the range.
     * @param x_max The maximum value of the range.
     * @param bits The number of bits used for representation.
     * @return The converted floating-point value.
     */
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    /**
     * @brief Converts a floating-point value to an integer value.
     * This static function performs data conversion provided by the DM motor.
     * @param x The floating-point value to be converted.
     * @param x_min The minimum value of the range.
     * @param x_max The maximum value of the range.
     * @param bits The number of bits used for representation.
     * @return The converted integer value.
     */
    static int float_to_uint(float x, float x_min, float x_max, int bits);

private:
    /**
     * @brief Set the CAN port number of this motor, and add it to the array of CAN ports.
     * @param port The port number.
     * @note This create a new feedback loop thread if the port is not in the array.
     */
    void set_port(int port);

    /**
     * @brief The feedback loop for the CAN driver.
     */
    void rx_loop(int port);
};

/**
 * @brief The DmMitDriver class represents a driver for a specific type of DM device.
 * This class inherits from the DmDriver base class and provides additional functionality
 * specific to the DmMitDriver. It allows setting the mode, parameters, velocity, and position
 * for the DM device.
 */
class DmMitDriver : public DmDriver
{
private:
    float kp, kd; ///< The proportional and derivative gains for the DM device.

public:
    /**
     * @brief Constructs a DmMitDriver object with the specified ID, kp, and kd values.
     * @param rid The ROS ID of the DM device.
     * @param hid The hardware ID of the DM device.
     * @param kp The proportional gain value.
     * @param kd The derivative gain value.
     */
    DmMitDriver(const std::string& rid, int hid, float kp, float kd, std::string port);

    /**
     * @brief Sets the mode of the DM device.
     * This function overrides the set_mode function from the base class.
     */
    void set_mode() override;

    /**
     * @brief Sets the proportional and derivative gains for the DM device.
     * @param kp The new proportional gain value.
     * @param kd The new derivative gain value.
     * @note This function also set t_ff to 1.0.
     */
    void set_param_mit(float kp, float kd);

    /**
     * @brief Sets the velocity goal for the DM device.
     * This function overrides the set_velocity function from the base class.
     * @param goal_vel The desired velocity.
     */
    void set_velocity(float goal_vel) override;

    /**
     * @brief Sets the position goal for the DM device.
     * This function overrides the set_position function from the base class.
     * @param goal_pos The desired position.
     */
    void set_position(float goal_pos) override;
};

/**
 * @brief The DmVelDriver class represents a velocity-based driver for a DM motor.
 * This class inherits from the DmDriver base class and provides specific implementations
 * for setting the mode, velocity, and position of the DM motor.
 */
class DmVelDriver : public DmDriver
{
public:
    /**
     * @brief Constructs a DmVelDriver object with the specified ID.
     * @param rid The ID of the DM motor.
     * @param hid The hardware ID of the DM motor.
     */
    DmVelDriver(const std::string& rid, int hid, std::string port);

    /**
     * @brief Sets the mode of the DM motor.
     * This function overrides the base class implementation and sets the mode of the DM motor
     * to velocity mode.
     */
    void set_mode() override;

    /**
     * @brief Sets the velocity of the DM motor.
     * This function overrides the base class implementation and sets the velocity of the DM motor
     * to the specified goal velocity.
     * @param goal_vel The goal velocity to set for the DM motor.
     */
    void set_velocity(float goal_vel) override;

    /**
     * @brief Sets the position of the DM motor.
     * This function overrides the base class implementation and sets the position of the DM motor
     * to the specified goal position.
     * @param goal_pos The goal position to set for the DM motor.
     */
    void set_position(float goal_pos) override;
};

#endif // DM_DRIVER_H