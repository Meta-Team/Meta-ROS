#include "unitree_controller/unitree_driver.hpp"
#include "unitreeMotor/unitreeMotor.h"
#include <cmath>
#include <thread>

UnitreeDriver::UnitreeDriver(std::string rid, int hid, std::string port, std::string type, int cali)
    : hid(hid), rid(rid), serial_port("/dev/tty" + port)
{
    if (type == "GO") this->type = MotorType::GO_M8010_6;
    else if (type == "A1") this->type = MotorType::A1;
    else if (type == "B1") this->type = MotorType::B1;
    else RCLCPP_ERROR(rclcpp::get_logger("unitree_driver"), "Unknown motor type: %s", type.c_str());

    goal_cmd.motorType = this->type;
    feedback_data.motorType = this->type;
    goal_cmd.mode = queryMotorMode(goal_cmd.motorType, MotorMode::FOC);
    goal_cmd.id = hid;
    stop();
    control_thread = std::thread(&UnitreeDriver::control_loop, this);
    cali_thread = std::thread(&UnitreeDriver::calibrate, this, cali);
}

UnitreeDriver::~UnitreeDriver()
{
    stop();
    if (control_thread.joinable()) control_thread.join();
    if (cali_thread.joinable()) cali_thread.join();
}

void UnitreeDriver::set_goal(double goal_pos, double goal_vel)
{
    if (!ready) return;
    goal_cmd.tau = 0.0; // cmd.T
    if (!std::isnan(goal_pos))
    {
        goal_cmd.kd = 0.0;
        goal_cmd.kp = this->kp;
        goal_cmd.q = goal_pos * queryGearRatio(type) + zero;
        goal_cmd.dq = 0.0;
    }
    if (!std::isnan(goal_vel))
    {
        goal_cmd.kd = this->kd;
        goal_cmd.kp = 0.0;
        goal_cmd.q = 0.0;
        goal_cmd.dq = goal_vel * queryGearRatio(type); // cmd.W
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
        (feedback_data.q - zero) / queryGearRatio(type), // pos
        feedback_data.dq / queryGearRatio(type), // vel
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
    while (rclcpp::ok())
    {
        this->send_recv();
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
    }
}

void UnitreeDriver::calibrate(int dir)
{
    auto log = rclcpp::get_logger("unitree_driver");

#define NOW rclcpp::Clock().now().seconds()
    const auto start = NOW;

    // if dir == 0, disable calibration
    if (dir == 0)
    {
        RCLCPP_INFO(log, "Motor %s zero calibration disabled", rid.c_str());
        zero = 0.0;
        // delay for a while
        ready = false;
        while (NOW - start < CALI_TIMEOUT && rclcpp::ok())
        {
            rclcpp::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
        }
        ready = true;
        return;
    }

    // if dir == 1 or -1, try to find zero
    bool found = false;
    auto last_not_jammed_moment = NOW;

    // try to find zero
    while (NOW - start < CALI_TIMEOUT && rclcpp::ok())
    {
        // set goal and get feedback
        goal_cmd.kd = this->kd;
        this->goal_cmd.dq = dir * TRY_VEL * queryGearRatio(type); // rad/s
        auto fb_vel = feedback_data.dq / queryGearRatio(type);

        // check if the motor is jammed
        if (std::abs(fb_vel) > TRY_VEL / 5) last_not_jammed_moment = NOW;

        // if jammed long enough
        if (NOW - last_not_jammed_moment > JAMMED_THRESHOLD)
        {
            found = true;
            break;
        }

        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
    }

    ready = true; // tempalily set to true to send goal
    if (found)
    {
        zero = feedback_data.q;
        set_goal(0.0, std::nan("")); // keep the motor still at zero
        RCLCPP_INFO(log, "Motor %s zero found: %f", rid.c_str(), zero);
    }
    else {
        zero = 0.0;
        set_goal(NaN, 0.0); // keep the motor still
        RCLCPP_WARN(log, "Motor %s zero not found", rid.c_str());
    }

    // delay for a while
    ready = false;
    while (NOW - start < CALI_TIMEOUT && rclcpp::ok())
    {
        rclcpp::sleep_for(std::chrono::milliseconds(UPDATE_FREQ));
    }
    ready = true;

#undef NOW
}