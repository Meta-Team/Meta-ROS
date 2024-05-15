#include "km_engineer/km_interpreter.h"

KmInterpreter::KmInterpreter(double vel, double aim_sens)
    : max_vel(vel), aim_sens(aim_sens)
{
    move_msg_ = std::make_shared<Move>();
    move_msg_->vel_x = 0.0;
    move_msg_->vel_y = 0.0;
    move_msg_->omega = 0.0;
    active = false;
    warned = false;

    last_op = 0.0;
    last_suck_toggle = 0.0;
    timeout_thread = std::thread(&KmInterpreter::check_timeout, this);
}

KmInterpreter::~KmInterpreter()
{
    if (timeout_thread.joinable()) timeout_thread.join();
}

void KmInterpreter::km_input(const KeyMouse::SharedPtr km_msg)
{
    active = km_msg->active;

    if (!active) return;

    last_op = rclcpp::Clock().now().seconds();
    move_msg_->vel_x = (km_msg->w - km_msg->s) * max_vel;
    move_msg_->vel_y = (km_msg->a - km_msg->d) * max_vel;
    move_msg_->omega = km_msg->mouse_x * aim_sens;
    // if (km_msg->q)
    // {
    //     if 
    // }

    if (warned)
    {
        warned = false;
        RCLCPP_INFO(rclcpp::get_logger("km_interpreter"), "KeyMouse operation resumed.");
    }
}

void KmInterpreter::check_timeout()
{
    // wait for receiving the first message
    rclcpp::sleep_for(std::chrono::seconds(1));

    // keep checking if the last operation is too old
    while (rclcpp::ok())
    {
        if (rclcpp::Clock().now().seconds() - last_op > 0.3)
        {
            move_msg_->vel_x = 0.0;
            move_msg_->vel_y = 0.0;
            move_msg_->omega = 0.0;
            if (!warned)
            {
                warned = true;
                RCLCPP_WARN(rclcpp::get_logger("km_interpreter"), "KeyMouse operation timeout.");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}