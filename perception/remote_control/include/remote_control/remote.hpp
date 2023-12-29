#ifndef REMOTE_HPP
#define REMOTE_HPP

#include <stdint.h>
#include <bitset>

/**
 * @class Remote
 * @brief A class to represent a remote control.
 * 
 * This class encapsulates the state of a remote control, including the state of the keys and the mouse.
 */
class Remote
{
public:
    /**
     * @enum Keys
     * @brief An enumeration to represent the keys on the remote control.
     * 
     * Each key on the remote control is represented by a member of this enumeration.
     */
    enum Keys
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

    int16_t mouse_x{}; ///< The x-coordinate of the mouse.
    int16_t mouse_y{}; ///< The y-coordinate of the mouse.
    int16_t mouse_z{}; ///< The z-coordinate of the mouse, the scroll wheel.

    int8_t left_button_down{}; ///< Represents the state of the left mouse button. Non-zero if the button is down.
    int8_t right_button_down{}; ///< Represents the state of the right mouse button. Non-zero if the button is down.

    /**
     * @union
     * @brief A union to represent the state of the keys.
     * 
     * The keys can be represented either as a 16-bit integer or as a bitset.
     */
    union {
        uint16_t key_uint{}; ///< The state of the keys, represented as a 16-bit unsigned integer.
        std::bitset<16> key_bits; ///< The state of the keys, represented as a bitset.
    };

    uint16_t reserved;
};

#endif // REMOTE_HPP