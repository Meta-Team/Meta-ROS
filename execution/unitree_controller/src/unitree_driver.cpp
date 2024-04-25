#include "unitree_controller/unitree_driver.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include <cmath>
#include <thread>

UnitreeDriver::UnitreeDriver(std::string rid, int hid)
    : hid(hid), rid(rid), serial_port("/dev/ttyUSB0")
{
    goal_cmd.motorType = MotorType::GO_M8010_6;
    feedback_data.motorType = MotorType::GO_M8010_6;
    goal_cmd.mode = queryMotorMode(goal_cmd.motorType, MotorMode::FOC);
    goal_cmd.id = hid;
    stop();
    update_thread = std::thread(&UnitreeDriver::control_loop, this);

#if CALI == true
    cali_thread = std::thread(&UnitreeDriver::calibrate, this);
#endif // CALI == ture
}

UnitreeDriver::~UnitreeDriver()
{
    stop();
    running = false;
    if (update_thread.joinable()) update_thread.join();
#if CALI == true
    if (cali_thread.joinable()) cali_thread.join();
#endif // CALI == true
}

void UnitreeDriver::set_goal(double goal_pos, double goal_vel)
{
    if (!ready) return;
    goal_cmd.tau = 0.0; // cmd.T
    if (!std::isnan(goal_pos))
    {
        goal_cmd.kd = 0.0;
        goal_cmd.kp = this->kp;
#if CALI == false
        goal_cmd.q = goal_pos * queryGearRatio(MotorType::GO_M8010_6); // cmd.Pos
#else // CALI == true
        goal_cmd.q = goal_pos * queryGearRatio(MotorType::GO_M8010_6) + zero;
#endif // CALI
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

void UnitreeDriver::set_pid(double kp, double kd)
{
    this->kp = kp;
    this->kd = kd;
}

void UnitreeDriver::send_recv()
{
    serial_port.sendRecv(&goal_cmd, &feedback_data);
}

std::tuple<double, double, double> UnitreeDriver::get_state()
{
    return std::make_tuple(
#if CALI == false
        feedback_data.q / queryGearRatio(MotorType::GO_M8010_6), // pos
#else // CALI == true
        (feedback_data.q - zero) / queryGearRatio(MotorType::GO_M8010_6), // pos
#endif // CALI
        feedback_data.dq / queryGearRatio(MotorType::GO_M8010_6), // vel
        feedback_data.tau // tor
    );
}

void UnitreeDriver::stop()
{
    goal_cmd.kp  = 0.0;
    goal_cmd.kd  = 0.0;
    goal_cmd.q   = 0.0;
    goal_cmd.dq  = 0.0;
    goal_cmd.tau = 0.0;
    send_recv();
}

void UnitreeDriver::control_loop()
{
    while (running)
    {
        this->send_recv();
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
    }
}

#if CALI == true
void UnitreeDriver::calibrate()
{
#define NOW rclcpp::Clock().now().seconds()

    bool found = false;
    const auto start = NOW;
    auto last_not_jammed_moment = NOW;

    while (rclcpp::Clock().now().seconds() - start < CALI_TIMEOUT)
    {
        // set goal and get feedback
        goal_cmd.kd = this->kd;
        this->goal_cmd.dq = TRY_VEL * queryGearRatio(MotorType::GO_M8010_6); // rad/s
        auto vel = feedback_data.dq / queryGearRatio(MotorType::GO_M8010_6);

        // tell whether jammed
        if (vel > TRY_VEL / 5) last_not_jammed_moment = NOW;

        // if jammed long enough
        if (NOW - last_not_jammed_moment > JAMMED_THRESHOLD)
        {
            found = true;
            break;
        }

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
    }

    if (found)
    {
        zero = feedback_data.q;
        set_goal(0.0, std::nan(""));
        auto log = rclcpp::get_logger("unitree_driver");
        RCLCPP_INFO(log, "Motor %s zero found: %f", rid.c_str(), zero);
    }
    else {
        auto log = rclcpp::get_logger("unitree_driver");
        RCLCPP_WARN(log, "Motor %s zero not found", rid.c_str());
    }
    ready = true;
}
#endif