#include "remote_vehicle/remote_interpreter.hpp"
#include <cmath>

RemoteInterpreter::RemoteInterpreter(double max_vel, double max_omega, double aim_sens, double interfere_sens)
    : max_vel(max_vel), max_omega(max_omega), aim_sensitive(aim_sens), interfere_sensitive(interfere_sens)
{
    // initialize buttons and axes
    w = a = s = d = shift = ctrl = q = e = r = f = g = z = x = c = v = b = false;
    left_button = right_button = false;
    mouse_x = mouse_y = mouse_z = 0;

    // initialize move, shoot, and aim
    move_ = std::make_shared<Move>();
    shoot_ = std::make_shared<Shoot>();
    aim_ = std::make_shared<Aim>();

    // start threads
    running = true;
    interpret_thread = std::thread([this] {
        while (running)
        {
            interpret();
            std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD));
        }
    });
}

RemoteInterpreter::~RemoteInterpreter()
{
    running = false;
    if (interpret_thread.joinable())
    {
        interpret_thread.join();
    }
}

void RemoteInterpreter::manual_input(const RemoteControl::SharedPtr msg)
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

void RemoteInterpreter::vision_input(const AutoAim::SharedPtr msg)
{
    auto_yaw = msg->yaw;
    auto_pitch = msg->pitch;
    last_auto_time = rclcpp::Clock().now().seconds();
}

void RemoteInterpreter::interpret()
{
    // move
    move_->vel_x = max_vel * (w - s);
    move_->vel_y = max_vel * (a - d);
    move_->omega = max_omega * (q - e);
    // shoot
    shoot_->fric_state = shift;
    shoot_->feed_state = left_button;
    // aim
    mode = right_button ? AUTO : MANUAL;
    if (mode == AUTO && rclcpp::Clock().now().seconds() - last_auto_time < 0.2)
    {
        manual_yaw += mouse_x * interfere_sensitive * PERIOD / 1000;
        manual_pitch += mouse_y * interfere_sensitive * PERIOD / 1000;
        aim_->yaw = auto_yaw + manual_yaw;
        aim_->pitch = auto_pitch + manual_pitch;
    }
    else // auto aim inactive, or timeout
    {
        aim_->yaw += mouse_x * aim_sensitive * PERIOD / 1000;
        aim_->pitch += mouse_y * aim_sensitive * PERIOD / 1000;
    }
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

void RemoteInterpreter::my_mod(double &val, double mod)
{
    val = fmod(val, mod);
    if (val < 0) val += mod;
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