#include "rclcpp/rclcpp.hpp"
#include "joy_vehicle/joy_interpreter.h"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>

class JoyVehicle : public rclcpp::Node
{
public:
    JoyVehicle() : Node("joy_vehicle")
    {
        // get param
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        double aim_sens = this->declare_parameter("control.aim_sensitivity", 1.57);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        RCLCPP_INFO(this->get_logger(), "max_vel: %f, max_omega: %f, aim_sens: %f, deadzone: %f",
            max_vel, max_omega, aim_sens, deadzone);
        
        interpreter_ = std::make_unique<JoyInterpreter>(max_vel, max_omega, aim_sens, deadzone);

        // pub and sub
        move_pub_ = this->create_publisher<behavior_interface::msg::Move>("move", 10);
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>("shoot", 10);
        aim_pub_ = this->create_publisher<behavior_interface::msg::Aim>("aim", 10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&JoyVehicle::joy_callback, this, std::placeholders::_1));

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PERIOD), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "JoyVehicle initialized.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    std::unique_ptr<JoyInterpreter> interpreter_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Publisher<behavior_interface::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        interpreter_->input(msg);
    }

    void timer_callback()
    {
        interpreter_->update();
        move_pub_->publish(*interpreter_->get_move());
        shoot_pub_->publish(*interpreter_->get_shoot());
        aim_pub_->publish(*interpreter_->get_aim());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<JoyVehicle>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}