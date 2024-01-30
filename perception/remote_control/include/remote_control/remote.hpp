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
#include "remote_control/remote_control.hpp"

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

public:
    /**
     * @brief Represents a remote frame structure used for communication.
     */
    struct __attribute__((packed, aligned(1))) RemoteFrame
    {
        /**
         * @brief Represents the header of the remote frame.
         */
        struct __attribute__((packed, aligned(1))) Header
        {
            uint8_t sof = 0xA5; /**< Start of Frame */
            uint16_t length; /**< Length of the frame */
            uint8_t seq; /**< Sequence number */
            uint8_t crc8; /**< CRC8 checksum */
        };

        /**
         * @brief Represents the data payload of the remote frame.
         */
        struct __attribute__((packed, aligned(1))) Data
        {
            int16_t mouse_x; /**< X-coordinate of the mouse */
            int16_t mouse_y; /**< Y-coordinate of the mouse */
            int16_t mouse_z; /**< Z-coordinate of the mouse */
            int8_t left_button; /**< Left button state */
            int8_t right_button; /**< Right button state */
            uint16_t keyboard; /**< Unsigned integer key value */
            uint16_t reserved; /**< Reserved field */
        };

        Header header; /**< The header of the remote frame */
        uint16_t command_id; /**< Command ID */
        Data data; /**< The data payload of the remote frame */
        uint16_t tail; /**< Tail of the frame */
    };

    // /**
    //  * @brief Reads a frame of data from the UART driver.
    //  * This function calls the read method of the UART driver to fill the frame vector with data.
    //  */
    // void rx_frame();

    // /**
    //  * @brief Processes the received frame.
    //  * This function checks the start of the frame, the command ID, and the length of the frame.
    //  * If these values are as expected, it extracts the state data from the frame.
    //  * @note Should be called after get_frame().
    //  */
    // void process_rx();

    // /**
    //  * @brief Sets the message based on the current state.
    //  * This function copies the current state data into the message object.
    //  * @note Should be called before publishing the message.
    //  */
    // operation_interface::msg::RemoteControl msg();
};

#endif // REMOTE_HPP