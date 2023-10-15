#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "can_driver.hpp"

#define I_MAX 16384

enum MotorType
{
    M3508,
    M2006,
    M6020,
};

class DjiDriver
{
private:
    int motor_id;
    MotorType motor_type;

    float present_pos;
    float former_pos;
    float present_vel;
    float former_vel;
    float current;

public:
    void update_pos(float pos);
    void update_vel(float vel);
    float vel2current(float goal_vel);
    float pos2current(float goal_pos);
    static void set_current(float &current, float goal);

    // linear mapping
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif // DJI_DRIVER_H