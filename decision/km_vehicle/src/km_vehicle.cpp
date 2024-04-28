#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
#include "km_vehicle/km_interpreter.hpp"

#include "operation_interface/msg/key_mouse.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "vision_interface/msg/auto_aim.hpp"

#define PUB_RATE 10 // Hz

using operation_interface::msg::KeyMouse;
using behavior_interface::msg::Move;
using behavior_interface::msg::Shoot;
using behavior_interface::msg::Aim;
using vision_interface::msg::AutoAim;

class KmVehicle : public rclcpp::Node
{
public:
    KmVehicle() : Node("km_vehicle")
    {
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        double aim_sens = this->declare_parameter("control.aim_sensitivity", 1.57);
        double interfere_sens = this->declare_parameter("control.interfere_sensitivity", 0.2);
        RCLCPP_INFO(this->get_logger(), "max_vel: %f, max_omega: %f, aim_sens: %f, interfere_sens: %f",
            max_vel, max_omega, aim_sens, interfere_sens);

        interpreter = std::make_unique<KmInterpreter>(max_vel, max_omega, aim_sens, interfere_sens);

        // pub and sub
        pub_move = this->create_publisher<Move>("move", 10);
        pub_shoot = this->create_publisher<Shoot>("shoot", 10);
        pub_aim = this->create_publisher<Aim>("aim", 10);
        km_sub_ = this->create_subscription<KeyMouse>(
            "key_mouse", 10,
            std::bind(&KmVehicle::km_callback, this, std::placeholders::_1));
        vision_sub_ = this->create_subscription<AutoAim>(
            "auto_aim", 10,
            std::bind(&KmVehicle::vision_callback, this, std::placeholders::_1));

        timer = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&KmVehicle::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "KmVehicle initialized");
    }

private:
    rclcpp::Subscription<KeyMouse>::SharedPtr km_sub_;
    rclcpp::Subscription<AutoAim>::SharedPtr vision_sub_;
    rclcpp::Publisher<Move>::SharedPtr pub_move;
    rclcpp::Publisher<Shoot>::SharedPtr pub_shoot;
    rclcpp::Publisher<Aim>::SharedPtr pub_aim;
    rclcpp::TimerBase::SharedPtr timer;

    std::unique_ptr<KmInterpreter> interpreter;

    void km_callback(const KeyMouse::SharedPtr msg)
    {
        interpreter->manual_input(msg);
    }

    void timer_callback()
    {
        if (!interpreter->is_active()) return;
        pub_move->publish(interpreter->get_move());
        pub_shoot->publish(interpreter->get_shoot());
        pub_aim->publish(interpreter->get_aim());
    }

    void vision_callback(const AutoAim::SharedPtr msg)
    {
        interpreter->vision_input(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmVehicle>());
    rclcpp::shutdown();
    return 0;
}