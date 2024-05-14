#include "rclcpp/rclcpp.hpp"

#include "km_engineer/km_interpreter.h"

#include "operation_interface/msg/key_mouse.hpp"
#include "behavior_interface/msg/grasp.hpp"
#include "behavior_interface/msg/move.hpp"
#include <memory>

#define PUB_RATE 20

class KmEngineer : public rclcpp::Node
{
public:
    KmEngineer() : Node("km_engineer")
    {
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double aim_sens = this->declare_parameter("control.mouse_sens", 1.57);
        RCLCPP_INFO(get_logger(), "max_vel: %f, aim_sens: %f", max_vel, aim_sens);

        move_pub_ = this->create_publisher<behavior_interface::msg::Move>("move", 10);
        grasp_pub_ = this->create_publisher<behavior_interface::msg::Grasp>("grasp", 10);
        key_mouse_sub_ = this->create_subscription<operation_interface::msg::KeyMouse>(
            "key_mouse", 10, std::bind(&KmEngineer::km_callback, this, std::placeholders::_1));

        interpreter_ = std::make_unique<KmInterpreter>(max_vel, aim_sens);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE), std::bind(&KmEngineer::pub_callback, this));

        RCLCPP_INFO(get_logger(), "KmEngineer initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::KeyMouse>::SharedPtr key_mouse_sub_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    rclcpp::Publisher<behavior_interface::msg::Grasp>::SharedPtr grasp_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::unique_ptr<KmInterpreter> interpreter_;


    void km_callback(const operation_interface::msg::KeyMouse::SharedPtr msg)
    {
        interpreter_->km_input(msg);
    }

    void pub_callback()
    {
        if (!interpreter_->is_active()) return;
        move_pub_->publish(interpreter_->get_move());
        grasp_pub_->publish(interpreter_->get_grasp());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmEngineer>());
    rclcpp::shutdown();
    return 0;
}