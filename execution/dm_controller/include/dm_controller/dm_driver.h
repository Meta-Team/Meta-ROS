#ifndef DM_DRIVER_H
#define DM_DRIVER_H

#include "can_driver.hpp"

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

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

/**
 * @brief The DmDriver class represents a driver for a DM motor.
 * 
 * This class provides functionality to control a DM motor, including turning it on and off,
 * setting the mode, velocity, and position of the motor, and performing data conversion.
 * 
 * The DmDriver class is an abstract base class, which means it cannot be instantiated directly.
 * Derived classes must implement the pure virtual functions set_mode(), set_velocity(), and set_position().
 * 
 * The DmDriver class also contains a static member variable can_0, which represents the CAN driver used for communication.
 * 
 * @note The DmDriver class assumes the use of the can_frame structure for CAN communication.
 */
class DmDriver
{
public:
    can_frame tx_frame; ///< The CAN frame used for transmitting data.
    int motor_id; ///< The ID of the motor controlled by the driver.

    /**
     * @brief Destructor for the DmDriver class.
     *
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

    static CanDriver* can_0; ///< The CAN driver used for communication.

    /**
     * @brief Sets the mode of the motor.
     * 
     * This function must be implemented by derived classes.
     */
    virtual void set_mode() = 0;

    /**
     * @brief Sets the velocity of the motor.
     * 
     * This function must be implemented by derived classes.
     * 
     * @param goal_vel The desired velocity of the motor.
     */
    virtual void set_velocity(float goal_vel) = 0;

    /**
     * @brief Sets the position of the motor.
     * 
     * This function must be implemented by derived classes.
     * 
     * @param goal_pos The desired position of the motor.
     */
    virtual void set_position(float goal_pos) = 0;

    /**
     * @brief Converts an integer value to a floating-point value.
     * 
     * This static function performs data conversion provided by the DM motor.
     * 
     * @param x_int The integer value to be converted.
     * @param x_min The minimum value of the range.
     * @param x_max The maximum value of the range.
     * @param bits The number of bits used for representation.
     * @return The converted floating-point value.
     */
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);

    /**
     * @brief Converts a floating-point value to an integer value.
     * 
     * This static function performs data conversion provided by the DM motor.
     * 
     * @param x The floating-point value to be converted.
     * @param x_min The minimum value of the range.
     * @param x_max The maximum value of the range.
     * @param bits The number of bits used for representation.
     * @return The converted integer value.
     */
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

/**
 * @brief The DmMitDriver class represents a driver for a specific type of DM device.
 * 
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
     * 
     * @param id The ID of the DM device.
     * @param kp The proportional gain value.
     * @param kd The derivative gain value.
     */
    DmMitDriver(int id, float kp, float kd);

    /**
     * @brief Sets the mode of the DM device.
     * 
     * This function overrides the set_mode function from the base class.
     */
    void set_mode() override;

    /**
     * @brief Sets the proportional and derivative gains for the DM device.
     * 
     * @param kp The new proportional gain value.
     * @param kd The new derivative gain value.
     *
     * @note This function also set t_ff to 1.0.
     */
    void set_param_mit(float kp, float kd);

    /**
     * @brief Sets the velocity goal for the DM device.
     * 
     * This function overrides the set_velocity function from the base class.
     * 
     * @param goal_vel The desired velocity.
     */
    void set_velocity(float goal_vel) override;

    /**
     * @brief Sets the position goal for the DM device.
     * 
     * This function overrides the set_position function from the base class.
     * 
     * @param goal_pos The desired position.
     */
    void set_position(float goal_pos) override;
};

/**
 * @brief The DmVelDriver class represents a velocity-based driver for a DM motor.
 * 
 * This class inherits from the DmDriver base class and provides specific implementations
 * for setting the mode, velocity, and position of the DM motor.
 */
class DmVelDriver : public DmDriver
{
public:
    /**
     * @brief Constructs a DmVelDriver object with the specified ID.
     * 
     * @param id The ID of the DM motor.
     */
    DmVelDriver(int id);

    /**
     * @brief Sets the mode of the DM motor.
     * 
     * This function overrides the base class implementation and sets the mode of the DM motor
     * to velocity mode.
     */
    void set_mode() override;

    /**
     * @brief Sets the velocity of the DM motor.
     * 
     * This function overrides the base class implementation and sets the velocity of the DM motor
     * to the specified goal velocity.
     * 
     * @param goal_vel The goal velocity to set for the DM motor.
     */
    void set_velocity(float goal_vel) override;

    /**
     * @brief Sets the position of the DM motor.
     * 
     * This function overrides the base class implementation and sets the position of the DM motor
     * to the specified goal position.
     * 
     * @param goal_pos The goal position to set for the DM motor.
     */
    void set_position(float goal_pos) override;
};

#endif // DM_DRIVER_H