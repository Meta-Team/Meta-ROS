#include "dji_controller/dji_driver.h"

void DjiDriver::update_pos(float pos)
{
    present_pos = pos;
    former_pos = present_pos;
}

void DjiDriver::update_vel(float vel)
{
    present_vel = vel;
    former_vel = present_vel;
}

void DjiDriver::set_current(float &current, float goal)
{
    if (goal != 0)
    {
        current = goal;
    }
    else {
        return;
    }
}

float DjiDriver::vel2current(float goal_vel)
{
    float current;

    return current;
}

float DjiDriver::pos2current(float goal_pos)
{
    float current;

    return current;
}