#ifndef DM_MOTOR_CONFIG_H
#define DM_MOTOR_CONFIG_H

#include "can_driver.h"

enum motor_mode{
    MIT_MODE,
    POS_VEL_MODE,
    VEL_MODE,
};

class DmMotorBase{
public:
    // CanDriver* can_driver;
    int        masterID;
    int        slaveID;
    float      mitKp;
    float      mitKd;
    float      V_max;   // maximum rotation speed. Unit is Rad/s.
    float      P_max;   // maximum Position. Unit is Rad.
    float      T_max;   // maximum Torque. Unit is N*m.
    float      initial_encoder_angle;
    motor_mode mode;
    float      kp_min;
    float      kp_max;
    float      kd_min;
    float      kd_max;
};

class DmMotorCFG{
public:
    enum MotorName{
        YAW,
        PITCH,
        MOTOR_COUNT,
    };
    static constexpr DmMotorBase motorCfg[MOTOR_COUNT] = {
            {0x00,0x01,1.0,0.3,30,3.141593,10.0,
             0.0,MIT_MODE,0.0,500.0,0.0,5.0}
    };
};

#endif // DM_MOTOR_CONFIG_H
