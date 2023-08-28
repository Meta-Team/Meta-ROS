#include "motor_driver.hpp"

class DjiMotorDriver : public MotorDriver
{
public:
    void turn_on() override;

    void turn_off() override;

    virtual void set_velocity(float goal_vel) override = 0;

    ~DjiMotorDriver() override;

private:
};

class DjiVelMotorDriver : public DjiMotorDriver
{
public:
    DjiVelMotorDriver(int name, int type);

    ~DjiVelMotorDriver() override;

    void set_mode() override;

    void set_velocity(float goal_vel) override;
};

class DjiPosMotorDriver : public DjiMotorDriver
{
public:
    DjiPosMotorDriver(int name, int type);

    ~DjiPosMotorDriver() override;

    void set_mode() override;

    void set_position(float goal_pos);
    
    void set_velocity(float goal_vel) override;
};
