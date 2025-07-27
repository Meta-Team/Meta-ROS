#include "hero_vehicle/dbus_interpreter.h"
#include <cmath>

DbusInterpreter::DbusInterpreter(double max_vel, double max_omega, double aim_sens, double deadzone, double video_link_blank_time)
    : max_vel(max_vel), max_omega(max_omega), aim_sens(aim_sens), deadzone(deadzone), video_link_blank_time(video_link_blank_time)
{
    // initialize buttons and axes
    active = false;
    keyboard_active_ = false;
    ls_x = ls_y = rs_x = rs_y = wheel = 0;
    lsw = rsw = "";

    // initialize move, shoot, aim, and chassis state
    move_ = std::make_shared<Move>();
    shoot_ = std::make_shared<Shoot>();
    aim_ = std::make_shared<Aim>();
    chassis_ = std::make_shared<Chassis>();

    // initialize chassis mode
    chassis_->mode = behavior_interface::msg::Chassis::CHASSIS;

    // Last Update Time
    last_update_time_ = rclcpp::Clock().now();
    last_c_ = false;

    // initialize update thread
    update_thread = std::thread([this](){
        while (rclcpp::ok())
        {
            update();
            std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD));
        }
    });
}

DbusInterpreter::~DbusInterpreter()
{
    if (update_thread.joinable()) update_thread.join();
}

void DbusInterpreter::input_dbus(const operation_interface::msg::DbusControl::SharedPtr msg)
{
    ls_x = msg->ls_x; apply_deadzone(ls_x); // forward is positive
    ls_y = msg->ls_y; apply_deadzone(ls_y); // left is positive
    rs_x = msg->rs_x; apply_deadzone(rs_x); // up is positive
    rs_y = msg->rs_y; apply_deadzone(rs_y); // left is positive
    wheel = msg->wheel; apply_deadzone(wheel);
    lsw = msg->lsw;
    rsw = msg->rsw;
}

void DbusInterpreter::input_video_link(const operation_interface::msg::KeyMouse::SharedPtr msg)
{
    keyboard_active_ = msg->active;
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
    left_button_ = msg->left_button;
    right_button_ = msg->right_button;
    mouse_x_ = msg->mouse_x;
    mouse_y_ = msg->mouse_y;
    // Video link will send packets even if no keys are pressed, except option panel(p) is active
    if(w_ || s_ || a_ || d_)
        last_video_link_recv_time = rclcpp::Clock().now();
}

void DbusInterpreter::update()
{
    active = (lsw == "MID") || keyboard_active_;
    if (!active)
    {
        move_->vel_x = 0.0;
        move_->vel_y = 0.0;
        move_->omega = 0.0;
        // printf("Update is not active\n");
        // TODO: aim ...
        return; // do not update if not active, this prevents yaw and pitch from accumulating in standby
    }

    move_->vel_x = max_vel * ls_x;
    move_->vel_y = max_vel * ls_y;
    aim_->pitch += aim_sens * rs_x * PERIOD / 1000; curb(aim_->pitch, M_PI_4);
    move_->omega = max_omega * wheel;
    aim_->yaw += aim_sens * rs_y * PERIOD / 1000;
    
    if (rsw == "UP")
    {
        shoot_->fric_state = false;
        shoot_->feed_state = false;
        shoot_->feed_speed = 0;
    }
    else if (rsw == "MID")
    {
        shoot_->fric_state = true;
        if(left_button_){   
            shoot_->feed_state = true;       
            shoot_->feed_speed = 5.0;
        }else{
            shoot_->feed_state = false; 
            shoot_->feed_speed = 0.0;
        }
    }
    else if (rsw == "DOWN")
    {
        shoot_->fric_state = true;
        shoot_->feed_state = true;
        shoot_->feed_speed = 5.0;
    }


    // TODO: Implement Keyboard Actions
    auto current_time = rclcpp::Clock().now();
    if (current_time.seconds() - last_video_link_recv_time.seconds() > video_link_blank_time) {
        // Video link timeout
        keyboard_active_ = false;
        kbd_move_x = 0.0;
        kbd_move_y = 0.0;
        // printf("Inactive video link\n");
    } else {
        if(w_) kbd_move_x += deadzone*0.1;
        if(s_) kbd_move_x -= deadzone*0.1;
        apply_deadzone(kbd_move_x);
        move_->vel_x += kbd_move_x*max_vel;

        if(a_) kbd_move_y += deadzone*0.1;
        if(d_) kbd_move_y -= deadzone*0.1;
        apply_deadzone(kbd_move_y);
        move_->vel_y += kbd_move_y*max_vel;

        aim_->yaw -= mouse_x_ * aim_sens * PERIOD / 200;   
        aim_->pitch -= mouse_y_ * aim_sens * PERIOD / 200;  curb(aim_->pitch, M_PI_4);
        if(q_) aim_->yaw += aim_sens * 0.5 * PERIOD / 1000;
        if(e_) aim_->yaw -= aim_sens * 0.5 * PERIOD / 1000;
    }
    // To ensure that the change take place only once per key press

    // if(current_time.seconds()-last_update_time_.seconds() > 0.2){
    //     if(c_ && !last_c_)  // TOGGLE CHASSIS MODE
    //     {
    //         if(chassis_->mode == behavior_interface::msg::Chassis::GYRO){
    //             chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
    //         }else if(chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW){
    //             chassis_->mode = behavior_interface::msg::Chassis::GYRO;
    //         }
    //     }
    //     last_update_time_ = rclcpp::Clock().now();
    // }

    // if(c_ && !last_c_){
    //     if(chassis_->mode == behavior_interface::msg::Chassis::GYRO){
    //         chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
    //     }else if(chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW){
    //         chassis_->mode = behavior_interface::msg::Chassis::GYRO;
    //     }
    // }
    last_c_ = c_;
    // if(ctrl_){
    //     chassis_->mode = behavior_interface::msg::Chassis::CHASSIS_FOLLOW;
    // }

    
    
}

void DbusInterpreter::apply_deadzone(double &val)
{
    if (val > deadzone ){
        val = deadzone;
    } else if (val < -deadzone){
        val = -deadzone;
    }
}

Move::SharedPtr DbusInterpreter::get_move() const
{
    return move_;
}

geometry_msgs::msg::Twist DbusInterpreter::get_move_ros2_control() const
{
    geometry_msgs::msg::Twist move_msg_ros2_control;
    move_msg_ros2_control.linear.x = move_->vel_x;
    move_msg_ros2_control.linear.y = move_->vel_y;
    move_msg_ros2_control.angular.z = move_->omega;
    return move_msg_ros2_control;
}

Shoot::SharedPtr DbusInterpreter::get_shoot() const
{
    return shoot_;
}

Aim::SharedPtr DbusInterpreter::get_aim() const
{
    return aim_;
}

Chassis::SharedPtr DbusInterpreter::get_chassis() const
{
    return chassis_;
}

void DbusInterpreter::curb(double &val, double max_val)
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