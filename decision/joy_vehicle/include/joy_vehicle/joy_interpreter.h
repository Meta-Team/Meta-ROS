#ifndef JOY_INTERPRETER_H
#define JOY_INTERPRETER_H

#include "rclcpp/rclcpp.hpp"
#include <thread>

#include "sensor_msgs/msg/joy.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"

#define PERIOD 10 // ms

using sensor_msgs::msg::Joy;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;

class JoyInterpreter
{
public:
    JoyInterpreter(float max_vel, float max_omega, float aim_sens, float deadzone);

    ~JoyInterpreter();

    void input(const Joy::SharedPtr msg);

    Move::SharedPtr get_move() const;

    Shoot::SharedPtr get_shoot() const;

    Aim::SharedPtr get_aim() const;

private:
    bool lb, rb, a, b, x, y, up, down, left, right;
    float lt, rt, ls_x, ls_y, rs_x, rs_y;

    float max_vel, max_omega, max_feed, max_shoot, aim_sens, deadzone;

    std::thread update_thread;

    Move::SharedPtr move_;
    Shoot::SharedPtr shoot_;
    Aim::SharedPtr aim_;

    void update();

    void apply_deadzone(float& val);

    void curb(double& val, double max_val);
};

#endif // JOY_INTERPRETER_H