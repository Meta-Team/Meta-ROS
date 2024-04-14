#include "unitree_controller/unitree_driver.hpp"
#include <rclcpp/logger.hpp>

UnitreeDriver::UnitreeDriver(std::string rid, int hid)
    : hid(hid), rid(rid), serial_port("/dev/ttyUSB0")
{
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
    if (!std::isnan(goal_pos))
    {
        goal_cmd.kd = 0.0;
        goal_cmd.kp = this->kp;
        goal_cmd.q = goal_pos * queryGearRatio(MotorType::GO_M8010_6); // cmd.Pos
        goal_cmd.dq = 0.0;
    }
    if (!std::isnan(goal_vel))
    {
        goal_cmd.kd = this->kd;
        goal_cmd.kp = 0.0;
        goal_cmd.q = 0.0;
        goal_cmd.dq = goal_vel * queryGearRatio(MotorType::GO_M8010_6); // cmd.W
    }
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

std::tuple<float, float, float> UnitreeDriver::get_state()
{
    return std::make_tuple(
        feedback_data.q / queryGearRatio(MotorType::GO_M8010_6), // pos
        feedback_data.dq / queryGearRatio(MotorType::GO_M8010_6), // vel
        feedback_data.tau // tor
    );
}