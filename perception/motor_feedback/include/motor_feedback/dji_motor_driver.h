#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"

#define POS_MAX 360 // degree
#define VEL_MAX 10000 // rpm, not sure
#define TOR_MAX 20 // actually current, A

enum MotorType
{
    M3508,
    M2006,
    M6020,
};

class DjiMotorDriver: public MotorDriver
{
public:
    MotorData process_rx() override;

private:
    MotorType motor_type;

    MotorData process_rx_3508();
    MotorData process_rx_2006();
    MotorData process_rx_6020();
};


#endif // DJI_MOTOR_DRIVER_H