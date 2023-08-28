#include "motor_driver.hpp"

class DmMotorDriver : public MotorDriver
{
public:
    void turn_on() override;
    
    void turn_off() override;

    ~DmMotorDriver() override;

private:
    constexpr static uint8_t start_cmd[8] = {0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    constexpr static uint8_t stop_cmd[8] = {0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    constexpr static uint8_t save_zero_cmd[8] = {0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    constexpr static uint8_t clear_error_cmd[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
};

class DmVelMotorDriver : public DmMotorDriver
{
public:
    DmVelMotorDriver(int name, int type);

    void set_mode() override;

    void set_velocity(float goal_vel) override;

    ~DmVelMotorDriver() override;
};

class DmMitMotorDriver : public DmMotorDriver
{
public:
    DmMitMotorDriver(int name, int type, float kp, float ki);

    void set_mode() override;

    void set_param_mit(float kp, float kd);

    void set_velocity(float goal_vel) override;

    void set_position(float goal_pos);

    void set_torque(float goal_torque);

    ~DmMitMotorDriver() override;
};