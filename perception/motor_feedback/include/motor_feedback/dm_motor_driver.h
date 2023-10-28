#ifndef DM_MOTOR_DRIVER_H
#define DM_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"

#define POS_MAX 3.14
#define VEL_MAX 3.14
#define TOR_MAX 1
// TODO: data to be changed

class DmMotorDriver : public MotorDriver
{
public:
    MotorData process_rx() override;
};

#endif // DM_MOTOR_DRIVER_H