#ifndef DM_MOTOR_DRIVER_H
#define DM_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"

#define P_MAX 12.5
#define V_MAX 30
#define T_MAX 10

class DmMotorDriver : public MotorDriver
{
public:
    MotorData process_rx() override;

    // this provided by DM
    float raw2actual(uint16_t raw, float actual_max, uint8_t bits)
    {
        return ((float)(raw - (2 << (bits - 2))) * 2 * actual_max)/(float)(2 << (bits - 1));
    }
};

#endif // DM_MOTOR_DRIVER_H