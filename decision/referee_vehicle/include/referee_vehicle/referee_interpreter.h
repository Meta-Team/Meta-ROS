#ifndef REFEREE_INTERPRETER_H
#define REFEREE_INTERPRETER_H

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <chrono>
#include <thread>

#include "operation_interface/msg/dbus_control.hpp"
#include "operation_interface/msg/key_mouse.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/chassis.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <behavior_interface/msg/detail/chassis__struct.hpp>

#define PERIOD 10 // ms

using operation_interface::msg::DbusControl;
using operation_interface::msg::KeyMouse;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;
using behavior_interface::msg::Chassis;

class RefereeInterpreter
{
public:
    RefereeInterpreter(double max_vel, double max_omega, double aim_sens, double deadzone);

    ~RefereeInterpreter();

    void dbus_input(const DbusControl::SharedPtr msg);

    void key_input(const KeyMouse::SharedPtr msg);

    Move::SharedPtr get_move() const;
    geometry_msgs::msg::Twist get_move_ros2_control() const;

    Shoot::SharedPtr get_shoot() const;

    Aim::SharedPtr get_aim() const;

    Chassis::SharedPtr get_chassis() const;

    bool is_active() const { return active; }

private:
    bool active;

    double ls_x, ls_y, rs_x, rs_y, wheel;
    double mouse_x_ = 0.0, mouse_y_ = 0.0, mouse_z_ = 0.0;
    bool left_button_ = false, right_button_ = false;
    bool w_ = false, a_ = false, s_ = false;
    bool d_ = false, shift_ = false, ctrl_ = false;
    bool q_ = false, e_ = false, r_ = false;
    bool f_ = false, g_ = false, z_ = false;
    bool x_ = false, c_ = false, v_, b_ = false;

    rclcpp::Time last_update_time_;

    std::string lsw, rsw;

    double max_vel, max_omega, max_feed, max_shoot, aim_sens, deadzone;

    std::thread update_thread;

    Move::SharedPtr move_;
    Shoot::SharedPtr shoot_;
    Aim::SharedPtr aim_;
    Chassis::SharedPtr chassis_;

    void update();

    void apply_deadzone(double& val);

    void curb(double& val, double max_val);
};

#endif // REFEREE_INTERPRETER_H