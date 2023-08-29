#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H

#include "motor_driver.hpp"

class DjiMotorDriver: public MotorDriver
{
public:
    ~DjiMotorDriver() override;
};


#endif // DJI_MOTOR_DRIVER_H