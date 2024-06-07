#include "auto_sentry/auto_decision.h"

AutoDecision::AutoDecision(double north_offset, double search_vel, double freq, double amplitude, double auto_rotate)
{
    this->north_offset = north_offset;
    this->search_vel = search_vel;
    this->freq = freq;
    this->amplitude = amplitude;
    this->auto_rotate = auto_rotate;

    active = false;

    aim_msg.yaw = north_offset;
    aim_msg.pitch = 0.0;

    move_msg.vel_x = 0.0;
    move_msg.vel_y = 0.0;
    move_msg.omega = auto_rotate;

    shoot_msg.id = 0; // MY_TODO: to be determined
    shoot_msg.fric_state = true;
    shoot_msg.feed_state = false;

    update_thread = std::thread(&AutoDecision::update_loop, this);
}

AutoDecision::~AutoDecision()
{
    if (update_thread.joinable()) update_thread.join();
}

Aim AutoDecision::get_aim() const
{
    return aim_msg;
}

Move AutoDecision::get_move() const
{
    return move_msg;
}

Shoot AutoDecision::get_shoot() const
{
    return shoot_msg;
}

void AutoDecision::vision_input(AutoAim::SharedPtr msg)
{
    last_vision_input = rclcpp::Clock().now().seconds();
    aim_msg.yaw = msg->yaw;
    aim_msg.pitch = msg->pitch;
    shoot_msg.feed_state = true;
}

void AutoDecision::dbus_input(DbusControl::SharedPtr msg)
{
    active = (msg->lsw == Switch::DOWN) ? true : false;
}

void AutoDecision::update_loop()
{
    while (rclcpp::ok())
    {
        double time_since_last_vision_input = rclcpp::Clock().now().seconds() - last_vision_input;
        if (time_since_last_vision_input > 0.05) shoot_msg.feed_state = false;
        if (time_since_last_vision_input > 0.2) search();

        rclcpp::sleep_for(std::chrono::milliseconds(UPDATE_R));
    }
}

void AutoDecision::search()
{
    // yaw
    static constexpr double dt = UPDATE_R / 1000.0;
    aim_msg.yaw += search_vel * dt;
    aim_msg.yaw = std::fmod(aim_msg.yaw, 2 * M_PI);

    // pitch sin wave
    aim_msg.pitch = amplitude * std::sin(freq * rclcpp::Clock().now().seconds());
}