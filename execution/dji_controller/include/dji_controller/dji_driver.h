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

    float p2v_kp, p2v_ki, p2v_kd;
    float v2c_kp, v2c_ki, v2c_kd;

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
    void set_p2v_pid(float kp, float ki, float kd);
    void set_v2c_pid(float kp, float ki, float kd);

    // linear mapping
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif // DJI_DRIVER_H