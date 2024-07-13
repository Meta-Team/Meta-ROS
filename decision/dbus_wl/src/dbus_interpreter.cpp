#include "dbus_wl/dbus_interpreter.h"
#include "rclcpp/rclcpp.hpp"

DbusInterpreter::DbusInterpreter(double max_vel, double max_omega, double deadzone, double height_sens)
    : active(false), max_vel(max_vel), max_omega(max_omega), deadzone(deadzone), height_sens(height_sens)
{
    // initialize buttons and axes
    ls_x = ls_y = rs_x = rs_y = wheel = 0;
    lsw = rsw = "";

    // initialize move
    move_ = std::make_shared<Move>();
    move_->vel_x = move_->vel_y = move_->omega = 0.0;
    move_->height = 0.2;

    // initialize update thread
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

void DbusInterpreter::input(const operation_interface::msg::DbusControl::SharedPtr msg)
{
    ls_x = msg->ls_x; apply_deadzone(ls_x); // forward is positive
    ls_y = msg->ls_y; apply_deadzone(ls_y); // left is positive
    rs_x = msg->rs_x; apply_deadzone(rs_x); // up is positive
    rs_y = msg->rs_y; apply_deadzone(rs_y); // left is positive
    wheel = msg->wheel; apply_deadzone(wheel);
    lsw = msg->lsw;
    rsw = msg->rsw;
}

Move::SharedPtr DbusInterpreter::get_move() const
{
    return move_;
}

void DbusInterpreter::update()
{
    active = (lsw == "MID");
    if (!active)
    {
        return; // do not update if not active, this prevents yaw and pitch from accumulating in standby
    }

    move_->vel_x = max_vel * ls_x;
    move_->vel_y = 0.0;
    move_->omega = max_omega * ls_y;
    move_->height += height_sens * rs_x * PERIOD / 1000.0;
    curb(move_->height, 0.3, 0.1); // 0.1m to 0.3m
}

void DbusInterpreter::apply_deadzone(double& val)
{
    if (std::abs(val) < deadzone) val = 0;
}

void DbusInterpreter::curb(double& val, double upper, double lower)
{
    if (val > upper) val = upper;
    if (val < lower) val = lower;
}