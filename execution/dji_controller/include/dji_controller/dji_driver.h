#ifndef DJI_DRIVER_H
#define DJI_DRIVER_H

#include "can_driver.hpp"
#include "dji_controller/motor_data.hpp"
#include <linux/can.h>
#include <memory>

#define ENCODER_ANGLE_RATIO 360.0f / 8192.0f
#define REDUCE_RATIO 36.0f

#define CONTROL_R 1 // ms
#define FEEDBACK_R 1 // ms

#define I_MAX 20 // Ampere
#define V_MAX 300 // to be tuned

enum MotorType
{
    M3508,
    M6020,
};

class DjiDriver
{
private:
    static std::unique_ptr<CanDriver> can_0;
    static can_frame tx_frame1, tx_frame2;
    static can_frame rx_frame;
    MotorType motor_type;

    PidParam p2v_prm, v2c_prm;
    PidOutput p2v_out, v2c_out;

    MotorData present_data;

    float goal_pos, goal_vel;
    float vel_error, pos_error;
    float current;

    void vel2current();
    void pos2velocity();

public:
    int motor_id;
    DjiDriver(int motor_id, MotorType motor_type);
    
    void set_goal(float goal_pos, float goal_vel);

    void set_p2v_pid(float kp, float ki, float kd);
    void set_v2c_pid(float kp, float ki, float kd);

    void write_frame();
    static void send_frame();
    static void get_frame();

    void process_rx();
};

#endif // DJI_DRIVER_H