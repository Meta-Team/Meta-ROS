#include "motor_feedback/dji_motor_driver.h"
#include "motor_feedback/motor_data.hpp"
#include "motor_feedback/motor_driver.hpp"

MotorData DjiMotorDriver::process_rx()
{
    switch (motor_type)
    {
    case M3508:
        return process_rx_3508();
    case M6020:
        return process_rx_6020();
    }
}

MotorData DjiMotorDriver::process_rx_3508()
{
    MotorData present_data;
    return present_data;
}

MotorData DjiMotorDriver::process_rx_6020()
{
    MotorData present_data;
    return present_data;
}