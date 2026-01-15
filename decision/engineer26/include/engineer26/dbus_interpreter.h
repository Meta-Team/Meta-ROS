#ifndef DBUS_INTERPRETER_H
#define DBUS_INTERPRETER_H

#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <thread>

#include "operation_interface/msg/dbus_control.hpp"
#include "operation_interface/msg/key_mouse.hpp"
#include "operation_interface/msg/vt03.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/chassis.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <behavior_interface/msg/detail/chassis__struct.hpp>

#define PERIOD 10 // ms

using operation_interface::msg::DbusControl;
using operation_interface::msg::KeyMouse;
using operation_interface::msg::VT03;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;
using behavior_interface::msg::Chassis;

class DbusInterpreter
{
public:
    DbusInterpreter(double max_vel, double max_omega, double aim_sens, double deadzone, double video_link_blank_time);

    ~DbusInterpreter();

    void input_dbus(const DbusControl::SharedPtr msg);

    Move::SharedPtr get_move() const;
    geometry_msgs::msg::Twist get_move_ros2_control() const;

    Chassis::SharedPtr get_chassis() const;
    double get_end_effector_velocity() const;
    bool is_active() const { return active; }

private:
    bool active;
    // video link
    rclcpp::Time last_video_link_recv_time;

    // dbus (DR16)
    double ls_x, ls_y, rs_x, rs_y, wheel;
    std::string lsw, rsw;


    rclcpp::Time last_update_time_;
    rclcpp::Time last_trigger_update_time_;

    double max_vel, max_omega, max_feed, max_shoot, aim_sens, deadzone, video_link_blank_time;

    std::thread update_thread;

    Move::SharedPtr move_;
    Chassis::SharedPtr chassis_;

    void update();

    void apply_deadzone(double& val);

    void curb(double& val, double max_val);
};

#endif // DBUS_INTERPRETER_H