#ifndef REMOTE_INTERPRETER_HPP
#define REMOTE_INTERPRETER_HPP

#include "operation_interface/msg/remote_control.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"
#include <cstdint>

#define PERIOD 10 // ms
#define PI 3.1415926f

using operation_interface::msg::RemoteControl;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;

class RemoteInterpreter
{
public:
    RemoteInterpreter(float max_vel, float max_omega, float aim_sens);
    
    void input(const RemoteControl::SharedPtr msg);

    void update();

    Move::SharedPtr get_move() const;

    Shoot::SharedPtr get_shoot() const;

    Aim::SharedPtr get_aim() const;

private:
    bool w, a, s, d, shift, ctrl, q, e, r, f, g, z, x, c, v, b;
    bool left_button, right_button;
    int16_t mouse_x, mouse_y, mouse_z;

    float max_vel, max_omega, max_feed, mac_shoot, aim_sensitive;

    Move::SharedPtr move_;
    Shoot::SharedPtr shoot_;
    Aim::SharedPtr aim_;

    void curb(double& val, double max_val);
};

#endif // REMOTE_INTERPRETER_HPP