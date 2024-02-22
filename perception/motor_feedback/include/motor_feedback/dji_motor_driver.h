#ifndef DJI_MOTOR_DRIVER_H
#define DJI_MOTOR_DRIVER_H

#include "motor_driver.hpp"
#include "motor_data.hpp"
#include <string>

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f

enum MotorType
{
    M3508,
    M6020,
};

class DjiMotorDriver: public MotorDriver
{
public:
    void process_rx() override;

    DjiMotorDriver(std::string rid, int hid, MotorType motor_type);
    
private:
    MotorType motor_type;
};


#endif // DJI_MOTOR_DRIVER_H