#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "auto_sentry/auto_decision.h"

#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "vision_interface/msg/auto_aim.hpp"
#include "operation_interface/msg/dbus_control.hpp"

#define PUB_R 10 // ms

using namespace behavior_interface::msg;
 
class AutoSentry : public rclcpp::Node
{
public:
    AutoSentry() : Node("auto_sentry")
    {
        aim_pub_ = this->create_publisher<Aim>("aim", 10);
        move_pub_ = this->create_publisher<Move>("move", 10);
        shoot_pub_ = this->create_publisher<Shoot>("shoot", 10);

        double search_vel = this->declare_parameter("gimbal.search_vel", 0.5);
        double freq = this->declare_parameter("gimbal.pitch_freq", 3.0);
        double amplitude = this->declare_parameter("gimbal.pitch_amp", 0.4);
        double north_offset = this->declare_parameter("north_offset", 0.0);
        double auto_rotate = this->declare_parameter("auto_rotate", 2.0);

        auto_decision_ = std::make_unique<AutoDecision>(north_offset, search_vel, freq, amplitude, auto_rotate);

        auto_aim_sub_ = this->create_subscription<AutoAim>("auto_aim", 10,
            std::bind(&AutoSentry::auto_aim_callback, this, std::placeholders::_1));
        dbus_control_sub_ = this->create_subscription<DbusControl>("dbus_control", 10,
            std::bind(&AutoSentry::dbus_control_callback, this, std::placeholders::_1));
        
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_R),
            std::bind(&AutoSentry::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "AutoSentry initialized");
    }

private:
    rclcpp::Publisher<Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<Move>::SharedPtr move_pub_;
    rclcpp::Publisher<Shoot>::SharedPtr shoot_pub_;
    rclcpp::Subscription<AutoAim>::SharedPtr auto_aim_sub_;
    rclcpp::Subscription<DbusControl>::SharedPtr dbus_control_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::unique_ptr<AutoDecision> auto_decision_;

    void timer_callback()
    {
        if (auto_decision_->is_active())
        {
            aim_pub_->publish(auto_decision_->get_aim());
            move_pub_->publish(auto_decision_->get_move());
            shoot_pub_->publish(auto_decision_->get_shoot());
        }
    }

    void auto_aim_callback(const AutoAim::SharedPtr msg)
    {
        auto_decision_->vision_input(msg);
    }

    void dbus_control_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        auto_decision_->dbus_input(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AutoSentry>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}