#include "motor_controller/dm_motor_driver.h"

#define START_CMD {0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define STOP_CMD {0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define SAVE_ZERO_CMD {0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
#define CLEAR_ERROR_CMD {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}

void DmMotorDriver::turn_on()
{
    uint8_t dlc_temp = tx_frame.can_dlc;
    for (int i = 10; i > 0; i --) {
        uint8_t start_cmd[8] = START_CMD;
        tx_frame.can_dlc = 0x08;
        memcpy(tx_frame.data, start_cmd, sizeof(start_cmd));
        can_0->send_frame(tx_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    can_0->send_frame(tx_frame);
    tx_frame.can_dlc = dlc_temp;
}

void DmMotorDriver::turn_off()
{
    uint8_t dlc_temp = tx_frame.can_dlc;
    for (int i = 10; i > 0; i --) {
        uint8_t stop_cmd[8] = STOP_CMD;
        tx_frame.can_dlc = 0x08;
        memcpy(tx_frame.data, stop_cmd, sizeof(stop_cmd));
        can_0->send_frame(tx_frame);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    can_0->send_frame(tx_frame);
    tx_frame.can_dlc = dlc_temp;
}

DmMotorDriver::~DmMotorDriver()
{
    turn_off();
}

// --------------------------------------------

void DmVelMotorDriver::set_mode()
{
    tx_frame.can_dlc = 0x04;
    tx_frame.can_id = motor_id + 0x200;
}

DmVelMotorDriver::DmVelMotorDriver(int name, int type)
{
    motor_id = name;
    motor_type = type;
    set_mode();
}

void DmVelMotorDriver::set_velocity(float goal_vel)
{
    float temp_vel = goal_vel;
    uint32_t* pvel;
    pvel = (uint32_t*) &temp_vel;
    memcpy(&tx_frame.data[0], pvel, sizeof(uint32_t));
    can_0->send_frame(tx_frame);
}

void DmVelMotorDriver::set_position(float goal_pos)
{
    // set_position invalid for velocity mode, do nothing
    return;
}

DmVelMotorDriver::~DmVelMotorDriver()
{
    turn_off();
}

// --------------------------------------------

DmMitMotorDriver::DmMitMotorDriver(int name, int type, float kp, float kd)
{
    motor_id = name;
    motor_type = type;
    set_mode();
    set_param_mit(kp, kd);
}

void DmMitMotorDriver::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = motor_id;
}

void DmMitMotorDriver::set_param_mit(float kp, float kd)
{
    uint32_t uint_kp = float_to_uint(kp, 0, 100, 12); // TODO
    uint32_t uint_kd = float_to_uint(kd, 0, 100, 12);
    tx_frame.data[3] &= 0xf0;
    tx_frame.data[3] |= (uint_kp >> 8) & 0x0f;
    tx_frame.data[4] = uint_kp & 0x0ff;
    tx_frame.data[6] &= 0x0f;
    tx_frame.data[6] |= (uint_kd & 0x0f) << 4;
    tx_frame.data[5] = uint_kd >> 4;
    can_0->send_frame(tx_frame);
}

void DmMitMotorDriver::set_velocity(float goal_vel)
{
    uint32_t uint_vel = float_to_uint(goal_vel,-100,100,12); // TODO: values to be changed
    tx_frame.data[3] &= 0x0f;
    tx_frame.data[3] |= (uint_vel & 0x0f) << 4;
    tx_frame.data[2] = uint_vel >> 4;
    can_0->send_frame(tx_frame);
}

void DmMitMotorDriver::set_position(float goal_pos)
{
    uint32_t  uint_pos = float_to_uint(goal_pos,-3.14,3.14,16);
    tx_frame.data[1] =uint_pos & 0x0ff;
    tx_frame.data[0] =uint_pos >> 8;
    can_0->send_frame(tx_frame);
}

void DmMitMotorDriver::set_torque(float goal_torque)
{
    uint32_t uint_torq = float_to_uint(goal_torque,-10,10,12);
    tx_frame.data[7] = uint_torq & 0x0ff;
    tx_frame.data[6] &= 0xf0;
    tx_frame.data[6] |= uint_torq >> 8;
    can_0->send_frame(tx_frame);
}

DmMitMotorDriver::~DmMitMotorDriver()
{
    turn_off();
}