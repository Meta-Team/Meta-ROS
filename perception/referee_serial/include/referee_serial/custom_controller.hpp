#ifndef CUSTOM_CONTROLLER_HPP
#define CUSTOM_CONTROLLER_HPP

#include <cstdint>
#include <memory>
#include <stdint.h>
#include <bitset>
#include <sys/types.h>
#include <vector>

#include "referee_serial/referee_serial.hpp"

class CustomController
{
public:
    struct [[gnu::packed]] FrameType
    {
        struct [[gnu::packed]] Header
        {
            uint8_t sof = 0xA5;
            uint16_t length;
            uint8_t seq;
        };
    };
};

#endif // CUSTOM_CONTROLLER_HPP