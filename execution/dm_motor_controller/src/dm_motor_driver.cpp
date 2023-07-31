#include "../include/dm_motor_controller/dm_motor_driver.h"
#include <bits/stdint-uintn.h>
#include <cstring>

motor_mode DmMotorDriver::motors_mode[DmMotorCFG::MotorName::MOTOR_COUNT];
CanDriver * DmMotorDriver::can_0;
can_frame DmMotorDriver::tx_frame[DmMotorCFG::MotorName::MOTOR_COUNT];
constexpr DmMotorBase DmMotorCFG::motorCfg[DmMotorCFG::MotorName::MOTOR_COUNT];
constexpr uint8_t DmMotorDriver::start_cmd[8];
constexpr uint8_t DmMotorDriver::stop_cmd[8];
constexpr uint8_t DmMotorDriver::save_zero_cmd[8];
constexpr uint8_t DmMotorDriver::clear_error_cmd[8];

void DmMotorDriver::init(CanDriver *can){

    can_0 = can;

    for(int i=0; i<DmMotorCFG::MOTOR_COUNT;i++){
        // tx_frame[i].RTR = CAN_RTR_DATA;
        // tx_frame[i].IDE = CAN_IDE_STD;
        if(DmMotorCFG::motorCfg[i].mode != VEL_MODE){
            tx_frame[i].can_dlc = 0x08;
        }else{
            tx_frame[i].can_dlc = 0x04;
        }
        motors_mode[i] = DmMotorCFG::motorCfg[i].mode;
    }
}

void DmMotorDriver::start(DmMotorCFG::MotorName motorProfile)
{
    DmMotorDriver::set_mode(motorProfile, DmMotorCFG::motorCfg[motorProfile].mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    for (int i = 10; i > 0; i --) {
        tx_frame[motorProfile].can_dlc = 0x08;
        memcpy(tx_frame[motorProfile].data, start_cmd, sizeof(start_cmd));
        can_0->send_frame(tx_frame[motorProfile]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (motors_mode[motorProfile] != VEL_MODE) {
        tx_frame[motorProfile].can_dlc = 0x08;
    }else {
        tx_frame[motorProfile].can_dlc = 0x04;
    }
}

void DmMotorDriver::stop(DmMotorCFG::MotorName motorProfile)
{
    // tx_frame[motorProfile].can_dlc = 0x08;
    memcpy(tx_frame[motorProfile].data, stop_cmd, sizeof(stop_cmd));
    can_0->send_frame(tx_frame[motorProfile]);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void DmMotorDriver::set_mode(DmMotorCFG::MotorName motorProfile, motor_mode mode)
{
    motors_mode[motorProfile] = mode;
    if (motors_mode[motorProfile] != MIT_MODE) {
        tx_frame[motorProfile].can_dlc = 0x08;
        tx_frame[motorProfile].can_id = DmMotorCFG::motorCfg[motorProfile].slaveID;
    }else if (motors_mode[motorProfile] == POS_VEL_MODE) {
        tx_frame[motorProfile].can_dlc = 0x08;
        tx_frame[motorProfile].can_id = DmMotorCFG::motorCfg[motorProfile].slaveID + 0x100;
    }else {
        tx_frame[motorProfile].can_dlc = 0x04;
        tx_frame[motorProfile].can_id = DmMotorCFG::motorCfg[motorProfile].slaveID + 0x200;
    }
}

void DmMotorDriver::set_velocity(DmMotorCFG::MotorName motorProfile, float vel)
{
    if (motors_mode[motorProfile] == MIT_MODE) {
        float v_max = DmMotorCFG::motorCfg[motorProfile].V_max;
        uint32_t uint_vel = float_to_uint(vel,-v_max,v_max,12);
        tx_frame[motorProfile].data[3] &= 0x0f;
        tx_frame[motorProfile].data[3] |= (uint_vel & 0x0f) << 4;
        tx_frame[motorProfile].data[2] = uint_vel >> 4;
    }else if (motors_mode[motorProfile] == POS_VEL_MODE) {
        float temp_vel = vel;
        uint32_t* pvel;
        pvel = (uint32_t*) &temp_vel;
        memcpy(&tx_frame[motorProfile].data[4], pvel, sizeof(uint32_t)); // tx_frame[motorProfile].data[4] is a pointer to the 5th element of the array
    }else {
        float temp_vel = vel;
        uint32_t* pvel;
        pvel = (uint32_t*) &temp_vel;
        memcpy(&tx_frame[motorProfile].data[0], pvel, sizeof(uint32_t));
    }
}

void DmMotorDriver::set_position(DmMotorCFG::MotorName motorProfile, float pos)
{
    if (motors_mode[motorProfile] == MIT_MODE) {
        float pos_max = DmMotorCFG::motorCfg[motorProfile].P_max;
        uint32_t  uint_pos = float_to_uint(pos,-pos_max,pos_max,16);
        tx_frame[motorProfile].data[1] =uint_pos & 0x0ff;
        tx_frame[motorProfile].data[0] =uint_pos >> 8;
    }else if (motors_mode[motorProfile] == POS_VEL_MODE) {
        float temp_pos = pos;
        uint32_t* ppos;
        ppos = (uint32_t*) &temp_pos;
        memcpy(&tx_frame[motorProfile].data[0], ppos, sizeof(uint32_t));
    }
}

void DmMotorDriver::set_torque(DmMotorCFG::MotorName motorProfile, float torque)
{
    if (motors_mode[motorProfile] == MIT_MODE) {
        float torque_max = DmMotorCFG::motorCfg[motorProfile].T_max;
        uint32_t uint_torq = float_to_uint(torque,-torque_max,torque_max,12);
        tx_frame[motorProfile].data[7] = uint_torq & 0x0ff;
        tx_frame[motorProfile].data[6] &= 0xf0;
        tx_frame[motorProfile].data[6] |= uint_torq >> 8;
    }
}

void DmMotorDriver::set_param_mit(DmMotorCFG::MotorName motorProfile, float kp, float kd)
{
    if (motors_mode[motorProfile] == MIT_MODE) {
        float kp_max = DmMotorCFG::motorCfg[motorProfile].kp_max;
        float kd_max = DmMotorCFG::motorCfg[motorProfile].kd_max;
        float kp_min = DmMotorCFG::motorCfg[motorProfile].kp_min;
        float kd_min = DmMotorCFG::motorCfg[motorProfile].kd_min;
        uint32_t uint_kp = float_to_uint(kp,kp_min,kp_max,12);
        uint32_t uint_kd = float_to_uint(kd,kd_min,kd_max,12);
        tx_frame[motorProfile].data[3] &= 0xf0;
        tx_frame[motorProfile].data[3] |= (uint_kp >> 8) & 0x0f;
        tx_frame[motorProfile].data[4] = uint_kp & 0x0ff;
        tx_frame[motorProfile].data[6] &= 0x0f;
        tx_frame[motorProfile].data[6] |= (uint_kd & 0x0f) << 4;
        tx_frame[motorProfile].data[5] = uint_kd >> 4;
    }
}
