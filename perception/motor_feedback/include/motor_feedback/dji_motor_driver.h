#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"


class DjiMotorDriver: public MotorDriver
{
public:
    MotorData process_rx() override;
};


#endif // DJI_MOTOR_DRIVER_H