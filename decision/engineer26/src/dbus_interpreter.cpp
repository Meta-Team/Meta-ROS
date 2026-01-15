#include "engineer26/dbus_interpreter.h"
#include <cmath>
#include "rclcpp/rclcpp.hpp"

DbusInterpreter::DbusInterpreter(double max_vel, double max_omega, double aim_sens, double deadzone,
                                 double video_link_blank_time) :
    max_vel(max_vel),
    max_omega(max_omega), aim_sens(aim_sens), deadzone(deadzone), video_link_blank_time(video_link_blank_time)
{
    // initialize buttons and axes
    active = false;
    ls_x = ls_y = rs_x = rs_y = wheel = 0;
    lsw = rsw = "";
    end_effector_pos = 0.0;
    // initialize move, shoot, aim, and chassis state
    chassis_ = std::make_shared<Chassis>();
    move_ = std::make_shared<Move>();

    // initialize chassis mode
    chassis_->mode = behavior_interface::msg::Chassis::CHASSIS;

    // Last Update Time
    last_trigger_update_time_ = last_update_time_ = rclcpp::Clock().now();

    // initialize update thread
    update_thread = std::thread(
        [this]()
        {
            while (rclcpp::ok())
            {
                update();
                std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD));
            }
        });
}

DbusInterpreter::~DbusInterpreter()
{
    if (update_thread.joinable())
        update_thread.join();
}

void DbusInterpreter::input_dbus(const operation_interface::msg::DbusControl::SharedPtr msg)
{
    ls_x = msg->ls_x;
    apply_deadzone(ls_x); // forward is positive
    ls_y = msg->ls_y;
    apply_deadzone(ls_y); // left is positive
    rs_x = msg->rs_x;
    apply_deadzone(rs_x); // up is positive
    rs_y = msg->rs_y;
    apply_deadzone(rs_y); // left is positive
    wheel = msg->wheel;
    apply_deadzone(wheel);
    lsw = msg->lsw;
    rsw = msg->rsw;
}


void DbusInterpreter::update()
{
    // 1. move chassis
    active = true;
    if ((lsw == "MID"))
    {
        move_->vel_x = max_vel * ls_x;
        move_->vel_y = max_vel * ls_y;
        move_->omega = wheel * max_omega;
    }else{
        move_->vel_x = 0.0;
        move_->vel_y = 0.0;
        move_->omega = 0.0;
    }
    // 2. 
    if ((lsw == "DOWN"))
    {
        end_effector_pos += wheel * 0.02; // don''t need to crop
    }
}

void DbusInterpreter::apply_deadzone(double& val)
{
    if (val > deadzone)
    {
        val = deadzone;
    }
    else if (val < -deadzone)
    {
        val = -deadzone;
    }
}

geometry_msgs::msg::Twist DbusInterpreter::get_move_ros2_control() const
{
    geometry_msgs::msg::Twist move_msg_ros2_control;
    move_msg_ros2_control.linear.x = move_->vel_x;
    move_msg_ros2_control.linear.y = move_->vel_y;
    move_msg_ros2_control.angular.z = move_->omega;
    return move_msg_ros2_control;
}

Chassis::SharedPtr DbusInterpreter::get_chassis() const { return chassis_; }
double DbusInterpreter::get_end_effector_position() const { return end_effector_pos; }

void DbusInterpreter::curb(double& val, double max_val)
{
    if (val > max_val)
    {
        val = max_val;
    }
    else if (val < -max_val)
    {
        val = -max_val;
    }
}

