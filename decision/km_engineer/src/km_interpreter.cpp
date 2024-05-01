#include "km_engineer/km_interpreter.h"
#include <rclcpp/utilities.hpp>

KmInterpreter::KmInterpreter(double vel, double aim_sens)
    : max_vel(vel), aim_sens(aim_sens)
{
    move_msg_ = std::make_shared<Move>();
    move_msg_->vel_x = 0.0;
    move_msg_->vel_y = 0.0;
    move_msg_->omega = 0.0;
    active = false;

    last_op = 0.0;
    timeout_thread = std::thread(&KmInterpreter::check_timeout, this);
}

void KmInterpreter::km_input(const KeyMouse::SharedPtr km_msg)
{
    last_op = rclcpp::Clock().now().seconds();
    active = km_msg->active;
    move_msg_->vel_x = (km_msg->w - km_msg->s) * max_vel;
    move_msg_->vel_y = (km_msg->a - km_msg->d) * max_vel;
    move_msg_->omega = km_msg->mouse_x * aim_sens;
}

void KmInterpreter::check_timeout()
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
            RCLCPP_WARN(rclcpp::get_logger("km_interpreter"), "Operation timeout.");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}