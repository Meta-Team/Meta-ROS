#include "referee_vehicle/referee_interpreter.h"
#include <behavior_interface/msg/detail/move__struct.hpp>
#include <behavior_interface/msg/detail/shoot__struct.hpp>
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

    last_update_time_ = rclcpp::Clock().now();
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

void RefereeInterpreter::dbus_input(const operation_interface::msg::DbusControl::SharedPtr msg)
{
    ls_x = msg->ls_x; apply_deadzone(ls_x); // forward is positive
    ls_y = msg->ls_y; apply_deadzone(ls_y); // left is positive
    rs_x = msg->rs_x; apply_deadzone(rs_x); // up is positive
    rs_y = msg->rs_y; apply_deadzone(rs_y); // left is positive
    wheel = msg->wheel; apply_deadzone(wheel);
    lsw = msg->lsw;
    rsw = msg->rsw;
}

void RefereeInterpreter::key_input(const KeyMouse::SharedPtr msg)
{
    w_ = msg->w;
    a_ = msg->a;
    s_ = msg->s;
    d_ = msg->d;
    shift_ = msg->shift;
    ctrl_ = msg->ctrl;
    q_ = msg->q;
    e_ = msg->e;
    r_ = msg->r;
    f_ = msg->f;
    g_ = msg->g;
    z_ = msg->z;
    x_ = msg->x;
    c_ = msg->c;
    v_ = msg->v;
    b_ = msg->b;
}

void RefereeInterpreter::update()
{
    active != (lsw == "MID" || lsw == "UP");        // Active only when the left switch in DOWN and default lsw == "" 
    if (!active)
    {
        return; // do not update if not active, this prevents yaw and pitch from accumulating in standby
    }

    // move_->vel_x = max_vel * ls_x;
    // move_->vel_y = max_vel * ls_y;
    // aim_->pitch += aim_sens * rs_x * PERIOD / 1000; curb(aim_->pitch, M_PI_4);
    // move_->omega = max_omega * wheel;
    // aim_->yaw += aim_sens * rs_y * PERIOD / 1000;
    // if(wheel > 0.01){      
    //     chassis_->mode = behavior_interface::msg::Chassis::CHASSIS;
    // }else{
    //     chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
    // }

    int move_x = 0, move_y = 0;
    if(a_) move_x -= max_vel;
    if(d_) move_x += max_vel;
    move_->vel_x += move_x;

    if(w_) move_y += max_vel;
    if(s_) move_y -= max_vel;
    move_->vel_y += move_y;

    if(left_button_){
        shoot_->feed_state = true;
        shoot_->feed_speed = 5.0;       // TODO: Modify this feed speed according to the level of the robot
    }else{  
        shoot_->feed_state = false;
        shoot_->feed_speed = 0.0;
    }

    aim_->yaw += mouse_x_ * 1.0 * PERIOD / 1000;   // TODO: Modify this ratio and test later
    aim_->pitch += mouse_y_ * 1.0 * PERIOD / 1000;  curb(aim_->pitch, M_PI_4);

    // To ensure that the change take place only once per key press
    // auto current_time = std::chrono::steady_clock::now();
    auto current_time = rclcpp::Clock().now();

    if(current_time.seconds()-last_update_time_.seconds() > 0.1){
        if(c_)  // TOGGLE CHASSIS MODE
        {
            if(chassis_->mode == behavior_interface::msg::Chassis::GYRO){
                chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
            }else{
                chassis_->mode = behavior_interface::msg::Chassis::GYRO;
            }
        }
        last_update_time_ = current_time = rclcpp::Clock().now();
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