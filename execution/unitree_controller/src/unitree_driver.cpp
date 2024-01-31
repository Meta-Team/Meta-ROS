#include "unitree_controller/unitree_driver.hpp"

UnitreeDriver::UnitreeDriver(int rid)
    : rid(rid), serial_port("/dev/ttyUSB0")
{
    hid = rid; // can be assigned with other values
    goal_cmd.motorType = MotorType::GO_M8010_6;
    feedback_data.motorType = MotorType::GO_M8010_6;
    goal_cmd.mode = queryMotorMode(goal_cmd.motorType, MotorMode::FOC);
    goal_cmd.id  = hid;
    goal_cmd.kp  = 0.0;
    goal_cmd.kd  = 0.0;
    goal_cmd.q   = 0.0 * queryGearRatio(MotorType::GO_M8010_6);
    goal_cmd.dq  = 0.0 * queryGearRatio(MotorType::GO_M8010_6);
    goal_cmd.tau = 0.0;
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

UnitreeDriver::~UnitreeDriver()
{
    goal_cmd.kp  = 0.0;
    goal_cmd.kd  = 0.0;
    goal_cmd.q   = 0.0;
    goal_cmd.dq  = 0.0;
    goal_cmd.tau = 0.0;
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

void UnitreeDriver::set_goal(float goal_pos, float goal_vel)
{
    goal_cmd.tau = 0.0; // cmd.T
    if (goal_pos != 0.0)
    {
        goal_cmd.kd = 0.0;
        goal_cmd.kp = this->kp;
        goal_cmd.q = goal_pos * queryGearRatio(MotorType::GO_M8010_6); // cmd.Pos
        goal_cmd.dq = 0.0;
    }
    if (goal_vel != 0.0)
    {
        goal_cmd.kd = this->kd;
        goal_cmd.kp = 0.0;
        goal_cmd.q = 0.0;
        goal_cmd.dq = goal_vel * queryGearRatio(MotorType::GO_M8010_6); // cmd.W
    }
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

void UnitreeDriver::set_pid(float kp, float kd)
{
    this->kp = kp;
    this->kd = kd;
}

void UnitreeDriver::send_recv()
{
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}