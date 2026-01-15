#include "rclcpp/rclcpp.hpp"
#include "engineer26/dbus_interpreter.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <memory>

#define PUB_RATE 15 // ms

class Engineer26 : public rclcpp::Node
{
public:
    Engineer26(const rclcpp::NodeOptions & options) : Node("engineer26")
    {
        // get param
        double max_vel = this->declare_parameter("control.trans_vel", 2.0);
        double max_omega = this->declare_parameter("control.rot_vel", 3.0);
        double aim_sens = this->declare_parameter("control.stick_sens", 1.57);
        double deadzone = this->declare_parameter("control.deadzone", 0.05);
        double video_link_blank_time = this->declare_parameter("control.video_link_blank_time", 0.1);
        std::string chassis_topic = this->declare_parameter("chassis_topic","chassis_cmd");
        RCLCPP_INFO(this->get_logger(), "max_vel: %f, max_omega: %f, aim_sens: %f, deadzone: %f",
            max_vel, max_omega, aim_sens, deadzone);

        interpreter_ = std::make_unique<DbusInterpreter>(max_vel, max_omega, aim_sens, deadzone, video_link_blank_time);

        // pub
        move_pub_ros2_control_ = this->create_publisher<geometry_msgs::msg::Twist>("omni_chassis_controller/reference", 10);
        chassis_pub_ = this->create_publisher<behavior_interface::msg::Chassis>(chassis_topic, 10);
        end_effector_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_end_effector_vel_controller/commands", 10);
        // sub from dbus_control
        dbus_sub_ = this->create_subscription<operation_interface::msg::DbusControl>(
            "dbus_control", 10,
            std::bind(&Engineer26::dbus_callback, this, std::placeholders::_1));
            
        // timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_RATE), [this](){
                timer_callback();
            });

        RCLCPP_INFO(this->get_logger(), "Engineer26 initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::DbusControl>::SharedPtr dbus_sub_;
    
    std::unique_ptr<DbusInterpreter> interpreter_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub_ros2_control_;
    rclcpp::Publisher<behavior_interface::msg::Chassis>::SharedPtr chassis_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr end_effector_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void dbus_callback(const operation_interface::msg::DbusControl::SharedPtr msg)
    {
        interpreter_->input_dbus(msg);
    }

    void timer_callback()
    {
        if (!interpreter_->is_active()) return;
        move_pub_ros2_control_->publish(interpreter_->get_move_ros2_control());
        chassis_pub_->publish(*interpreter_->get_chassis());
        auto msg_end_effector_velocity = std_msgs::msg::Float64MultiArray();
        msg_end_effector_velocity.data.push_back(interpreter_->get_end_effector_velocity());
        end_effector_publisher_->publish(msg_end_effector_velocity);
    }
};
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Engineer26)
