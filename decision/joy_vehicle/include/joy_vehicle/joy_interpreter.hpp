#ifndef JOY_INTERPRETER_HPP
#define JOY_INTERPRETER_HPP

#include "sensor_msgs/msg/joy.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"

#define PERIOD 10 // ms
#define PI 3.1415926f

using sensor_msgs::msg::Joy;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;

class JoyInterpreter
{
public:
    JoyInterpreter(float max_vel, float max_omega, float aim_sens, float deadzone);

    void input(const Joy::SharedPtr msg);

    void update();

    Move::SharedPtr get_move() const;

    Shoot::SharedPtr get_shoot() const;

    Aim::SharedPtr get_aim() const;

private:
    bool lb, rb, a, b, x, y, up, down, left, right;
    float lt, rt, ls_x, ls_y, rs_x, rs_y;

    float max_vel, max_omega, max_feed, max_shoot, aim_sensitive, deadzone;

    Move::SharedPtr move_;
    Shoot::SharedPtr shoot_;
    Aim::SharedPtr aim_;

    void apply_deadzone(float& val);

    void curb(double& val, double max_val);
};

#endif // JOY_INTERPRETER_HPP