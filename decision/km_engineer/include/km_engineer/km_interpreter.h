#ifndef KM_INTERPRETER_H
#define KM_INTERPRETER_H

#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/key_mouse.hpp"
#include "behavior_interface/msg/move.hpp"

using behavior_interface::msg::Move;
using operation_interface::msg::KeyMouse;

class KmInterpreter
{
public:
    KmInterpreter(double vel, double aim_sens);

    void km_input(const KeyMouse::SharedPtr km_msg);

    Move get_move() { return *move_msg_; }

    bool is_active() { return active; }

private:
    bool active;
    Move::SharedPtr move_msg_;

    double max_vel;
    double aim_sens;

    double last_op;
    std::thread timeout_thread;

    void check_timeout();
};

#endif // KM_INTERPRETER_H