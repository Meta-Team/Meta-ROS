#include "dbus_arm/dbus_interpreter.h"
#include <behavior_interface/msg/detail/end_vel__struct.hpp>
#include <rclcpp/utilities.hpp>

DbusInterpreter::DbusInterpreter(double linear, double angular, double deadzone)
    : linear_v(linear), angular_v(angular), deadzone(deadzone)
{
    ls_x = ls_y = rs_x = rs_y = 0;

    // initialize end_vel
    end_vel_ = std::make_shared<EndVel>();

    // start update thread
    update_thread = std::thread([this](){
        while (rclcpp::ok())
        {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD));
        }
    });
}

DbusInterpreter::~DbusInterpreter()
{
    if (update_thread.joinable()) update_thread.join();
}

void DbusInterpreter::input(const DbusControl msg)
{
    ls_x = msg.ls_x; apply_deadzone(ls_x); // forward is positive
    ls_y = msg.ls_y; apply_deadzone(ls_y); // left is positive
    rs_x = msg.rs_x; apply_deadzone(rs_x); // up is positive
    rs_y = msg.rs_y; apply_deadzone(rs_y); // left is positive
}

void DbusInterpreter::update()
{
    end_vel_->x = linear_v * ls_x;
    end_vel_->y = linear_v * ls_y;
    end_vel_->z = linear_v * rs_x;
    end_vel_->yaw = angular_v * rs_y;
}

EndVel::SharedPtr DbusInterpreter::get_end_vel() const
{
    return end_vel_;
}

void DbusInterpreter::apply_deadzone(double &val)
{
    if (val < deadzone && val > -deadzone)
    {
        val = 0;
    }
}

void DbusInterpreter::curb(double &val, double max_val)
{
    if (val > max_val) val = max_val;
    if (val < -max_val) val = -max_val;
}