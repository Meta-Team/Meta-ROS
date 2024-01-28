#ifndef REMOTE_HPP
#define REMOTE_HPP

#include <cstdint>
#include <memory>
#include <operation_interface/msg/detail/remote_control__struct.hpp>
#include <stdint.h>
#include <bitset>
#include <sys/types.h>
#include <vector>

#include "operation_interface/msg/remote_control.hpp"
#include "remote_control/uart_driver.hpp"

/**
 * @class Remote
 * @brief A class to represent a remote control.
 * 
 * This class encapsulates the state of a remote control, including the state of the keys and the mouse.
 */
class Remote
{
private:
    /**
     * @enum Keys
     * @brief An enumeration to represent the keys on the remote control.
     * Can be used to access the bitset "key_bits".
     */
    enum // Keys
    {
        W = 0,
        S = 1,
        A = 2,
        D = 3,
        SHIFT = 4,
        CTRL = 5,
        Q = 6,
        E = 7,
        R = 8,
        F = 9,
        G = 10,
        Z = 11,
        X = 12,
        C = 13,
        V = 14,
        B = 15,
    };

    static std::unique_ptr<UartDriver> uart_; ///< A pointer to the UART driver.
    std::vector<uint8_t> frame; ///< A vector to store the UART frame.

    // variables used to store the state of the remote control
    int16_t mouse_x{}; ///< The x-coordinate of the mouse.
    int16_t mouse_y{}; ///< The y-coordinate of the mouse.
    int16_t mouse_z{}; ///< The z-coordinate of the mouse, the scroll wheel.
    int8_t left_button{}; ///< Represents the state of the left mouse button. Non-zero if the button is down.
    int8_t right_button{}; ///< Represents the state of the right mouse button. Non-zero if the button is down.
    union {
        uint16_t key_uint{}; // The state of the keys, represented as a 16-bit unsigned integer.
        std::bitset<16> key_bits; // The state of the keys, represented as a bitset.
    } keyboard{}; ///< The state of the keys.

public:
    /**
     * @brief Reads a frame of data from the UART driver.
     * This function calls the read method of the UART driver to fill the frame vector with data.
     */
    void rx_frame();

    /**
     * @brief Processes the received frame.
     * This function checks the start of the frame, the command ID, and the length of the frame.
     * If these values are as expected, it extracts the state data from the frame.
     * @note Should be called after get_frame().
     */
    void process_rx();

    /**
     * @brief Sets the message based on the current state.
     * This function copies the current state data into the message object.
     * @note Should be called before publishing the message.
     */
    operation_interface::msg::RemoteControl msg();
};

#endif // REMOTE_HPP