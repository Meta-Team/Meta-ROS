#include "rclcpp/rclcpp.hpp"
#include "hero_vehicle/dbus_interpreter.h"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

#define PUB_RATE 15 // ms

class HeroVehicle : public rclcpp::Node
{
public:
    HeroVehicle(const rclcpp::NodeOptions & options) : Node("hero_vehicle")
    {
        // get param
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        std::string aim_topic = this->declare_parameter("aim_topic", "aim");
        std::string shoot_topic = this->declare_parameter("shoot_topic", "shoot");
        std::string chassis_topic = this->declare_parameter("chassis_topic","chassis_cmd");
        RCLCPP_INFO(this->get_logger(), "max_vel: %f, max_omega: %f, aim_sens: %f, deadzone: %f",
            max_vel, max_omega, aim_sens, deadzone);
        enable_key_mouse_ = this->declare_parameter("enable_key_mouse_", true);
        enable_ros2_control_ = this->declare_parameter("enable_ros2_control", false);

        interpreter_ = std::make_unique<DbusInterpreter>(max_vel, max_omega, aim_sens, deadzone);

        // pub and sub
        if (enable_ros2_control_) {
            move_pub_ros2_control_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "omni_chassis_controller/reference", 10);
        } else {
            move_pub_ = this->create_publisher<behavior_interface::msg::Move>("move", 10);
        }
        shoot_pub_ = this->create_publisher<behavior_interface::msg::Shoot>(shoot_topic, 10);
        aim_pub_ = this->create_publisher<behavior_interface::msg::Aim>(aim_topic, 10);
        chassis_pub_ = this->create_publisher<behavior_interface::msg::Chassis>(chassis_topic, 10);
        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10,
            std::bind(&HeroVehicle::dbus_callback, this, std::placeholders::_1));
        if(enable_key_mouse_){
            key_sub_ = this->create_subscription<operation_interface::msg::KeyMouse>(
                "key_mouse", 10,
                std::bind(&HeroVehicle::key_callback, this, std::placeholders::_1));
        }

        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "Hero Vehicle initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    rclcpp::Subscription<operation_interface::msg::KeyMouse>::SharedPtr key_sub_;
    std::unique_ptr<DbusInterpreter> interpreter_;
    rclcpp::Publisher<behavior_interface::msg::Move>::SharedPtr move_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub_ros2_control_;
    rclcpp::Publisher<behavior_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Publisher<behavior_interface::msg::Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<behavior_interface::msg::Chassis>::SharedPtr chassis_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool enable_ros2_control_, enable_key_mouse_;

    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        interpreter_->input(msg);
    }

    void key_callback(const operation_interface::msg::KeyMouse::SharedPtr msg)
    {
        interpreter_->input_key(msg);
    }

    void timer_callback()
    {
        if (!interpreter_->is_active()) return;
        if (enable_ros2_control_) {
            move_pub_ros2_control_->publish(interpreter_->get_move_ros2_control());
        } else {
            move_pub_->publish(*interpreter_->get_move());
        }
        shoot_pub_->publish(*interpreter_->get_shoot());
        aim_pub_->publish(*interpreter_->get_aim());
        chassis_pub_->publish(*interpreter_->get_chassis());
    }
};
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(HeroVehicle)