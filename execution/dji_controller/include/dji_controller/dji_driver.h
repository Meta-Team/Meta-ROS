#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "can_driver.hpp"
#include <linux/can.h>

#define I_MAX 16384

#define DT 0.01

enum MotorType
{
    M3508,
    M2006,
    M6020,
};

class DjiDriver // TODO: constructor and destructor
{
private:
    static CanDriver* can_0;

    int motor_id;
    MotorType motor_type;

    float p2v_kp, p2v_ki, p2v_kd;
    float v2c_kp, v2c_ki, v2c_kd;
    float proportional, integral, derivative;

    float present_pos;
    float present_vel;
    float goal_pos;
    float goal_vel;
    float current;
    float vel_error;
    float pos_error;

public:
    void set_goal(float goal_pos, float goal_vel);
    void update_pos(float pos);
    void update_vel(float vel);
    float vel2current(float goal_vel);
    float pos2velocity(float goal_pos);
    float pos2current(float goal_pos);
    void set_p2v_pid(float kp, float ki, float kd);
    void set_v2c_pid(float kp, float ki, float kd);

    void write_frame(can_frame &tx_frame);
    static void send_frame(can_frame &tx_frame);

    // linear mapping
    static float uint_to_float(int x_int, float x_min, float x_max, int bits);
    static int float_to_uint(float x, float x_min, float x_max, int bits);
};

#endif // DJI_DRIVER_H