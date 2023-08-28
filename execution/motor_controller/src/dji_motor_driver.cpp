#include "../include/motor_controller/dji_motor_driver.h"

void DjiMotorDriver::turn_on()
{
    // TODO
}

void DjiMotorDriver::turn_off()
{
    // TODO
}

DjiMotorDriver::~DjiMotorDriver()
{
    turn_off();
}

// --------------------------------------------

DjiVelMotorDriver::DjiVelMotorDriver(int name, int type)
{
    motor_id = name;
    motor_type = type;
    set_mode();
}

DjiVelMotorDriver::~DjiVelMotorDriver()
{
    turn_off();
}

void DjiVelMotorDriver::set_mode()
{
    // TODO
}

void DjiVelMotorDriver::set_velocity(float goal_vel)
{
    // TODO
}

void DjiVelMotorDriver::set_position(float goal_pos)
{
    // set_position invalid for velocity mode, do nothing
    return;
}

// --------------------------------------------

DjiPosMotorDriver::DjiPosMotorDriver(int name, int type)
{
    motor_id = name;
    motor_type = type;
    set_mode();
}

DjiPosMotorDriver::~DjiPosMotorDriver()
{
    turn_off();
}

void DjiPosMotorDriver::set_mode()
{
    // TODO
}

void DjiPosMotorDriver::set_position(float goal_pos)
{
    // TODO
}

void DjiPosMotorDriver::set_velocity(float goal_vel)
{
    // TODO
}