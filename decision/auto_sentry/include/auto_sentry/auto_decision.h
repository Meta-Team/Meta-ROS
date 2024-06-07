#ifndef AUTO_DECISION_H
#define AUTO_DECISION_H

#include "rclcpp/rclcpp.hpp"

#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "operation_interface/msg/dbus_control.hpp"
#include "vision_interface/msg/auto_aim.hpp"

using namespace behavior_interface::msg;
using operation_interface::msg::DbusControl;
using vision_interface::msg::AutoAim;

#define UPDATE_R 5 // ms

enum Switch
{
    UP = 1,
    MID = 3,
    DOWN = 2
};

class AutoDecision
{
public:
    AutoDecision(double north_offset, double search_vel, double freq, double amplitude, double auto_rotate);
    ~AutoDecision();

    Aim get_aim() const;
    Move get_move() const;
    Shoot get_shoot() const;

    void vision_input(AutoAim::SharedPtr msg);

    void dbus_input(DbusControl::SharedPtr msg);

    bool is_active() const { return active; }

private:
    bool active;
    double last_vision_input = 0.0; // the last time vision input is received

    Aim aim_msg;
    Move move_msg;
    Shoot shoot_msg;

    double north_offset = 0.0;
    double search_vel = 0.5; // rad/s
    double freq = 3.0;
    double amplitude = 0.4; // rad
    double auto_rotate = 2; // rad/s

    std::thread update_thread;

    void update_loop();

    void search();
};

#endif // AUTO_DECISION_H