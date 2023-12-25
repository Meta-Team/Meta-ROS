#include "dm_controller/dm_driver.h"
#include <cstdio>

/***********************************************
 * DmDriver is the base class for all drivers.
 ***********************************************/

std::unique_ptr<CanDriver> DmDriver::can_0 = std::make_unique<CanDriver>(0);

float DmDriver::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// Converts an unsigned int to a float, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (float) x_int * span / (float) ((1<<bits)-1) + offset;
}

int DmDriver::float_to_uint(float x, float x_min, float x_max, int bits){
    /// Converts a float to an unsigned int, given range and number of bits
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

void DmDriver::turn_on()
{
    auto frame_temp = tx_frame;

    uint8_t start_cmd[8] = START_CMD;
    tx_frame.can_dlc = 0x08;
    memcpy(tx_frame.data, start_cmd, sizeof(start_cmd));
    can_0->send_frame(tx_frame);
        
    printf("turn-on frame sent\n");
    tx_frame = frame_temp;
}

void DmDriver::turn_off()
{
    uint8_t dlc_temp = tx_frame.can_dlc;

    uint8_t stop_cmd[8] = STOP_CMD;
    tx_frame.can_dlc = 0x08;
    memcpy(tx_frame.data, stop_cmd, sizeof(stop_cmd));
    can_0->send_frame(tx_frame);

    printf("turn-off frmae sent\n");
    tx_frame.can_dlc = dlc_temp;
}

DmDriver::~DmDriver()
{
    turn_off();
    printf("DmDriver deleted\n");
}

/*************************************************************************************
 * DmMitDriver is a subclass of DmDriver that represents a specific type of driver.
 ************************************************************************************/

DmMitDriver::DmMitDriver(int name, float kp, float kd)
{
    printf("DmMitDriver created\n");
    this->motor_id = name;
    set_mode();
    set_param_mit(kp, kd);
}

void DmMitDriver::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = motor_id;
}

void DmMitDriver::set_param_mit(float kp, float kd)
{
    float tff = 1;
    uint32_t uint_kp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    uint32_t uint_kd = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    uint32_t uint_tff = float_to_uint(tff, -T_MAX, T_MAX, 12);
    tx_frame.data[3] &= 0xf0;
    tx_frame.data[3] |= (uint_kp >> 8) & 0x0f;
    tx_frame.data[4] = uint_kp & 0x0ff;
    tx_frame.data[5] = uint_kd >> 4;
    tx_frame.data[6] &= 0x0f;
    tx_frame.data[6] |= (uint_kd & 0x0f) << 4;
    tx_frame.data[6] |= (uint_tff >> 8) & 0x0f;
    tx_frame.data[7] = uint_tff & 0x0ff;
    set_velocity(1);
}

void DmMitDriver::set_velocity(float goal_vel)
{
    uint32_t uint_vel = float_to_uint(goal_vel, -V_MAX, V_MAX, 12);
    tx_frame.data[3] &= 0x0f;
    tx_frame.data[3] |= (uint_vel & 0x0f) << 4;
    tx_frame.data[2] = uint_vel >> 4;
    can_0->send_frame(tx_frame);
}

void DmMitDriver::set_position(float goal_pos)
{
    uint32_t uint_pos = float_to_uint(goal_pos, -P_MAX, P_MAX, 16);
    tx_frame.data[1] = uint_pos & 0x0ff;
    tx_frame.data[0] = uint_pos >> 8;
    can_0->send_frame(tx_frame);
    printf("frame sent: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", tx_frame.data[i]);
    }
    printf("\n");
}

/*************************************************************************************
 * DmVelDriver is a subclass of DmDriver that represents a specific type of driver.
 ************************************************************************************/

DmVelDriver::DmVelDriver(int name)
{
    this->motor_id = name;
    set_mode();
}

void DmVelDriver::set_mode()
{
    tx_frame.can_dlc = 0x08;
    tx_frame.can_id = motor_id;
}

void DmVelDriver::set_velocity(float goal_vel)
{
    float temp_vel = goal_vel;
    uint32_t* pvel;
    pvel = (uint32_t*) &temp_vel;
    memcpy(&tx_frame.data[0], pvel, sizeof(uint32_t));
    can_0->send_frame(tx_frame);
}

void DmVelDriver::set_position(float /*goal_pos*/)
{
    // set_position invalid for velocity mode, do nothing
    return;
}