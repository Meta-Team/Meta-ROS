#include "joy_arm/joy_interpreter.hpp"
#include <rclcpp/utilities.hpp>

JoyInterpreter::JoyInterpreter(double linear, double angular, double deadzone)
    : linear_v(linear), angular_v(angular), deadzone(deadzone)
{
    // initialize buttons and axes
    lb = rb = a = b = x = y = up = down = left = right = false;
    lt = rt = ls_x = ls_y = rs_x = rs_y = 0;

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

JoyInterpreter::~JoyInterpreter()
{
    if (update_thread.joinable()) update_thread.join();
}

void JoyInterpreter::input(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    ls_x = msg->axes[1]; apply_deadzone(ls_x); // forward is positive
    ls_y = msg->axes[0]; apply_deadzone(ls_y); // left is positive
    rs_x = msg->axes[4]; apply_deadzone(rs_x); // up is positive
    rs_y = msg->axes[3]; apply_deadzone(rs_y); // left is positive
    lt = msg->axes[2];
    rt = msg->axes[5];
    lb = static_cast<bool>(msg->buttons[4]);
    rb = static_cast<bool>(msg->buttons[5]);
    a = static_cast<bool>(msg->buttons[0]);
    b = static_cast<bool>(msg->buttons[1]);
    x = static_cast<bool>(msg->buttons[2]);
    y = static_cast<bool>(msg->buttons[3]);
    
    // MY_TODO: implement up, down, left, right
}

void JoyInterpreter::update()
{
    end_vel_->x = linear_v * ls_x;
    end_vel_->y = linear_v * ls_y;
    end_vel_->z = linear_v * ((int)lb - (int)rb);
    end_vel_->pitch = angular_v * rs_x;
    end_vel_->yaw = angular_v * rs_y;
    end_vel_->roll = angular_v * (lt - rt);
}

void JoyInterpreter::apply_deadzone(double &val)
{
    if (val < deadzone && val > -deadzone)
    {
        val = 0;
    }
}

EndVel::SharedPtr JoyInterpreter::get_end_vel() const
{
    return end_vel_;
}

void JoyInterpreter::curb(double &val, double max_val)
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