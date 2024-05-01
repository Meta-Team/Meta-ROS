#include "dbus_engineer/dbus_interpreter.h"

DbusInterpreter::DbusInterpreter(double vel, double aim_sens)
    : max_vel(vel), aim_sens(aim_sens)
{
    move_msg_ = std::make_shared<Move>();
    move_msg_->vel_x = 0.0;
    move_msg_->vel_y = 0.0;
    move_msg_->omega = 0.0;
    active = false;

    last_op = 0.0;
    timeout_thread = std::thread(&DbusInterpreter::check_timeout, this);
}

DbusInterpreter::~DbusInterpreter()
{
    if (timeout_thread.joinable()) timeout_thread.join();
}

void DbusInterpreter::dbus_input(const DbusControl::SharedPtr dbus_msg)
{
    last_op = rclcpp::Clock().now().seconds();
    active = dbus_msg->rsw == 2 ? false : true; // MY_TODO: check the value
    move_msg_->vel_x = dbus_msg->ls_x * max_vel;
    move_msg_->vel_y = dbus_msg->ls_y * max_vel;
    move_msg_->omega = dbus_msg->rs_x * aim_sens;
}

void DbusInterpreter::check_timeout()
{
    // wait for receiving the first message
    rclcpp::sleep_for(std::chrono::seconds(1));

    // keep checking if the last operation is too old
    while (rclcpp::ok())
    {
        if (rclcpp::Clock().now().seconds() - last_op > 0.5)
        {
            move_msg_->vel_x = 0.0;
            move_msg_->vel_y = 0.0;
            move_msg_->omega = 0.0;
            RCLCPP_WARN(rclcpp::get_logger("dbus_interpreter"), "Operation timeout.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}