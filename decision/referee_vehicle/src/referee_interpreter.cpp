#include "referee_vehicle/referee_interpreter.h"
#include <cmath>

RefereeInterpreter::RefereeInterpreter(double max_vel, double max_omega, double aim_sens, double deadzone)
    : max_vel(max_vel), max_omega(max_omega), aim_sens(aim_sens), deadzone(deadzone)
{
    // initialize buttons and axes
    active = false;
    ls_x = ls_y = rs_x = rs_y = wheel = 0;
    lsw = rsw = "";

    // initialize move, shoot, aim, and chassis state
    move_ = std::make_shared<Move>();
    shoot_ = std::make_shared<Shoot>();
    aim_ = std::make_shared<Aim>();
    chassis_ = std::make_shared<Chassis>();

    // initialize update thread
    update_thread = std::thread([this](){
        while (rclcpp::ok())
        {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD));
        }
    });
}

RefereeInterpreter::~RefereeInterpreter()
{
    if (update_thread.joinable()) update_thread.join();
}

void RefereeInterpreter::input(const operation_interface::msg::DbusControl::SharedPtr msg)
{
    ls_x = msg->ls_x; apply_deadzone(ls_x); // forward is positive
    ls_y = msg->ls_y; apply_deadzone(ls_y); // left is positive
    rs_x = msg->rs_x; apply_deadzone(rs_x); // up is positive
    rs_y = msg->rs_y; apply_deadzone(rs_y); // left is positive
    wheel = msg->wheel; apply_deadzone(wheel);
    lsw = msg->lsw;
    rsw = msg->rsw;
}

void RefereeInterpreter::update()
{
    active = (lsw == "MID");
    if (!active)
    {
        return; // do not update if not active, this prevents yaw and pitch from accumulating in standby
    }

    move_->vel_x = max_vel * ls_x;
    move_->vel_y = max_vel * ls_y;
    aim_->pitch += aim_sens * rs_x * PERIOD / 1000; curb(aim_->pitch, M_PI_4);
    move_->omega = max_omega * wheel;
    aim_->yaw += aim_sens * rs_y * PERIOD / 1000;
    if(wheel > 0.01){      
        chassis_->mode = behavior_interface::msg::Chassis::CHASSIS;
    }else{
        chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
    }
    
    
    
    if (rsw == "UP")
    {
        shoot_->fric_state = false;
        shoot_->feed_state = false;
        shoot_->feed_speed = 0;
    }
    else if (rsw == "MID")
    {
        shoot_->fric_state = true;
        shoot_->feed_state = false;
        shoot_->feed_speed = 0;
    }
    else if (rsw == "DOWN")
    {
        shoot_->fric_state = true;
        shoot_->feed_state = true;
        shoot_->feed_speed = -4.0;
    }
}

void RefereeInterpreter::apply_deadzone(double &val)
{
    if (val < deadzone && val > -deadzone)
    {
        val = 0;
    }
}

Move::SharedPtr RefereeInterpreter::get_move() const
{
    return move_;
}

geometry_msgs::msg::Twist RefereeInterpreter::get_move_ros2_control() const
{
    geometry_msgs::msg::Twist move_msg_ros2_control;
    move_msg_ros2_control.linear.x = move_->vel_x;
    move_msg_ros2_control.linear.y = move_->vel_y;
    move_msg_ros2_control.angular.z = move_->omega;
    return move_msg_ros2_control;
}

Shoot::SharedPtr RefereeInterpreter::get_shoot() const
{
    return shoot_;
}

Aim::SharedPtr RefereeInterpreter::get_aim() const
{
    return aim_;
}

Chassis::SharedPtr RefereeInterpreter::get_chassis() const
{
    return chassis_;
}

void RefereeInterpreter::curb(double &val, double max_val)
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