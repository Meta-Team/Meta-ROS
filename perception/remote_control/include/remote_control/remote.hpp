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
     * Can be used to interpret the keyboard field of the remote frame.
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
    struct [[gnu::packed]] RemoteFrame
    {
        /**
         * @brief Represents the header of the remote frame.
         */
        struct [[gnu::packed]] Header
        {
            static constexpr uint8_t sof = 0xA5; /**< Start of Frame */
            uint16_t length; /**< Length of the frame */
            uint8_t seq; /**< Sequence number */
            uint8_t crc8; /**< CRC8 checksum */
        };

        /**
         * @brief Represents the data payload of the remote frame.
         */
        struct [[gnu::packed]] Data
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
        uint16_t cmd_id; /**< Command ID */
        Data data; /**< The data of the remote frame */
        uint16_t tail; /**< Tail of the frame */
    };

    RemoteFrame interpreted; /**< The interpreted frame */ 

    /**
     * @brief Checks if the prefix is a wanted prefix.
     * @param prefix The prefix to check.
     * @return True if the prefix is a wanted prefix, false otherwise.
     */
    static bool is_wanted_pre(const std::vector<uint8_t> &prefix);

    /**
     * @brief Constructs a Remote object from a frame.
     * This would interpret the frame by copying the data into the RemoteFrame struct.
     * @param frame The frame to construct the Remote object from.
     */
    Remote(const std::vector<uint8_t> &frame);

    /**
     * @brief Sets the message based on the current state.
     * This function copies the current state data into the message object.
     * @note Should be called before publishing the message.
     */
    operation_interface::msg::RemoteControl msg();
};

#endif // REMOTE_HPP