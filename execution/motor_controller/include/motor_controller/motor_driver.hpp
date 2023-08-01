#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "cstdint"
#include "can_driver.hpp"
#include <bits/stdint-uintn.h>
#include <linux/can.h>
#include <chrono>
#include <thread>

#define VEL_MODE 0
#define POS_MODE 1
#define MIT_MODE 1

// TODO: add other motor params

class MotorDriver
{
public:
    int motor_id;
    int motor_type;
    can_frame tx_frame;
    CanDriver* can_0;

    virtual void set_mode(int mode) = 0;
    virtual void turn_on() = 0;
    virtual void set_velocity(float goal_vel) = 0;
    virtual void turn_off() = 0;

    virtual ~MotorDriver() = default;

    static float uint_to_float(int x_int, float x_min, float x_max, int bits){
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
    }

    static int float_to_uint(float x, float x_min, float x_max, int bits){
        /// Converts a float to an unsigned int, given range and number of bits
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
    }
};

// ---------------------DaMiao-----------------------

class DmMotorDriver : public MotorDriver
{
public:
    void set_mode(int mode) override
    {
        if (mode == VEL_MODE)
        {
            tx_frame.can_dlc = 0x04;
            tx_frame.can_id = motor_id + 0x200;
        } else if (mode == MIT_MODE)
        {
            tx_frame.can_dlc = 0x08;
            tx_frame.can_id = motor_id;
        }
    }
    void turn_on() override
    {
        uint8_t dlc_temp = tx_frame.can_dlc;
        for (int i = 10; i > 0; i --) {
            tx_frame.can_dlc = 0x08;
            memcpy(tx_frame.data, start_cmd, sizeof(start_cmd));
            can_0->send_frame(tx_frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        tx_frame.can_dlc = dlc_temp;
    }
    void turn_off() override
    {
        uint8_t dlc_temp = tx_frame.can_dlc;
        for (int i = 10; i > 0; i --) {
            tx_frame.can_dlc = 0x08;
            memcpy(tx_frame.data, stop_cmd, sizeof(stop_cmd));
            can_0->send_frame(tx_frame);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        tx_frame.can_dlc = dlc_temp;
    }

    ~DmMotorDriver()
    {
        turn_off();
    }

private:
    static constexpr uint8_t start_cmd[8] = {0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t stop_cmd[8] = {0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t save_zero_cmd[8] = {0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t clear_error_cmd[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
};

class DmVelMotorDriver : public DmMotorDriver
{
public:
    DmVelMotorDriver(int name, int type)
    {
        motor_id = name;
        motor_type = type;
        set_mode(VEL_MODE);
    }
    void set_velocity(float goal_vel) override;
};

class DmMitMotorDriver : public DmMotorDriver
{
public:
    DmMitMotorDriver(int name, int type, float kp, float ki)
    {
        motor_id = name;
        motor_type = type;
        set_mode(MIT_MODE);
        set_param_mit(kp, ki);
    }
    void set_param_mit(float kp, float kd)
    {
        uint32_t uint_kp = float_to_uint(kp, 0, 100, 12); // TODO
        uint32_t uint_kd = float_to_uint(kd, 0, 100, 12);
        tx_frame.data[3] &= 0xf0;
        tx_frame.data[3] |= (uint_kp >> 8) & 0x0f;
        tx_frame.data[4] = uint_kp & 0x0ff;
        tx_frame.data[6] &= 0x0f;
        tx_frame.data[6] |= (uint_kd & 0x0f) << 4;
        tx_frame.data[5] = uint_kd >> 4;
    }
    void set_velocity(float goal_vel) override; // TODO
    void set_position(float goal_pos);
};

// ---------------------DJI-----------------------

class DjiMotorDriver : public MotorDriver
{
public:
    void set_mode(int mode) override
    {
        if (mode == VEL_MODE)
        {
            tx_frame.can_dlc = 0x04;
            tx_frame.can_id = motor_id + 0x200;
        } else if (mode == POS_MODE)
        {
            tx_frame.can_dlc = 0x08;
            tx_frame.can_id = motor_id;
        }
    }
    void turn_on() override
    {
        // TODO
    }
    void turn_off() override
    {
        // TODO
    }
    ~DjiMotorDriver()
    {
        turn_off();
    }

private:
    static constexpr uint8_t start_cmd[8] = {0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t stop_cmd[8] = {0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t save_zero_cmd[8] = {0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    static constexpr uint8_t clear_error_cmd[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
};

class DjiVelMotorDriver : public DjiMotorDriver
{
public:
    DjiVelMotorDriver(int name, int type)
    {
        motor_id = name;
        motor_type = type;
        set_mode(VEL_MODE);
    }
    void set_velocity(float goal_vel) override;
};

class DjiPosMotorDriver : public DjiMotorDriver
{
public:
    DjiPosMotorDriver(int name, int type)
    {
        motor_id = name;
        motor_type = type;
        set_mode(POS_MODE);
    }
    void set_position(float goal_pos);
    void set_velocity(float goal_vel) override;
};

#endif // MOTOR_DRIVER_HPP