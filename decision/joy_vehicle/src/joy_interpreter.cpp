#include "joy_vehicle/joy_interpreter.hpp"

JoyInterpreter::JoyInterpreter(float max_vel, float max_omega, float aim_sens, float deadzone)
    : max_vel(max_vel), max_omega(max_omega), aim_sensitive(aim_sens), deadzone(deadzone)
{
    // initialize buttons and axes
    lb = rb = a = b = x = y = up = down = left = right = false;
    lt = rt = ls_x = ls_y = rs_x = rs_y = 0;

    // initialize move, shoot, and aim
    move_ = std::make_shared<Move>();
    shoot_ = std::make_shared<Shoot>();
    aim_ = std::make_shared<Aim>();
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
    move_->vel_x = max_vel * ls_x;
    move_->vel_y = max_vel * ls_y;
    move_->omega = max_omega * (rt - lt) / 2;
    aim_->pitch += aim_sensitive * rs_x * PERIOD / 1000; curb(aim_->pitch, PI / 2);
    aim_->yaw += aim_sensitive * rs_y * PERIOD / 1000;
    shoot_->fric_state = lb;
    shoot_->feed_state = rb;
}

void JoyInterpreter::apply_deadzone(float &val)
{
    if (val < deadzone && val > -deadzone)
    {
        val = 0;
    }
}

Move::SharedPtr JoyInterpreter::get_move() const
{
    return move_;
}

Shoot::SharedPtr JoyInterpreter::get_shoot() const
{
    return shoot_;
}

Aim::SharedPtr JoyInterpreter::get_aim() const
{
    return aim_;
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