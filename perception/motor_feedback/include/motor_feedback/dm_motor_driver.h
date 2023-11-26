#ifndef DM_MOTOR_DRIVER_H
#define DM_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"

#define P_MAX 3.141593
#define V_MAX 30
#define T_MAX 10

class DmMotorDriver : public MotorDriver
{
public:
    MotorData process_rx() override;

    float uint_to_float(int x_int, float x_min, float x_max, int bits);

    int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif // DM_MOTOR_DRIVER_H