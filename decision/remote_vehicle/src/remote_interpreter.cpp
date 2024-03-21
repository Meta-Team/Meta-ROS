#include "remote_vehicle/remote_interpreter.hpp"

RemoteInterpreter::RemoteInterpreter(float max_vel, float max_omega, float aim_sens)
    : max_vel(max_vel), max_omega(max_omega), aim_sensitive(aim_sens)
{
    // initialize buttons and axes
    w = a = s = d = shift = ctrl = q = e = r = f = g = z = x = c = v = b = false;
    left_button = right_button = false;
    mouse_x = mouse_y = mouse_z = 0;

    // initialize move, shoot, and aim
    move_ = std::make_shared<Move>();
    shoot_ = std::make_shared<Shoot>();
    aim_ = std::make_shared<Aim>();
}

void RemoteInterpreter::input(const RemoteControl::SharedPtr msg)
{
    w = msg->w;
    a = msg->a;
    s = msg->s;
    d = msg->d;
    shift = msg->shift;
    ctrl = msg->ctrl;
    q = msg->q;
    e = msg->e;
    r = msg->r;
    f = msg->f;
    g = msg->g;
    z = msg->z;
    x = msg->x;
    c = msg->c;
    v = msg->v;
    b = msg->b;
    left_button = msg->left_button;
    right_button = msg->right_button;
    mouse_x = msg->mouse_x;
    mouse_y = msg->mouse_y;
    mouse_z = msg->mouse_z;
}

void RemoteInterpreter::update()
{
    move_->vel_x = max_vel * (w - s);
    move_->vel_y = max_vel * (a - d);
    move_->omega = max_omega * (q - e);
    aim_->pitch += aim_sensitive * mouse_y * PERIOD / 1000; curb(aim_->pitch, PI / 2);
    aim_->yaw += aim_sensitive * mouse_x * PERIOD / 1000;
    shoot_->fric_state = right_button;
    shoot_->feed_state = left_button;
}

void RemoteInterpreter::curb(double &val, double max_val)
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

Move::SharedPtr RemoteInterpreter::get_move() const
{
    return move_;
}

Shoot::SharedPtr RemoteInterpreter::get_shoot() const
{
    return shoot_;
}

Aim::SharedPtr RemoteInterpreter::get_aim() const
{
    return aim_;
}