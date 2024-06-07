#include "rclcpp/rclcpp.hpp"

#include "omni_chassis/omni_kinematics.hpp"

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_state.hpp"
#include "device_interface/msg/motor_goal.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <ctime>
#include <memory>

#define PUB_RATE 10 // ms

class OmniChassis : public rclcpp::Node
{
public:
    OmniChassis() : Node("OmniChassis")
    {
        // get params
        std::string move_mode = this->declare_parameter("move_mode", "chassis");
        double cha_r = this->declare_parameter("chassis.chassis_radius", 0.3);
        double wheel_r = this->declare_parameter("chassis.wheel_radius", 0.05);
        double decel_ratio = this->declare_parameter("chassis.deceleration_ratio", 20.0);
        double n_offset = this->declare_parameter("north_offset", 0.0);
        double yaw_offset = this->declare_parameter("chassis.yaw_offset", 0.0);
        kine = std::make_unique<OmniKinematics>(wheel_r, cha_r, decel_ratio, n_offset, yaw_offset);

        // init publisher and subscribers
        motor_pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
        motor_sub_ = this->create_subscription<device_interface::msg::MotorState>("motor_state",
            10, std::bind(&OmniChassis::motor_callback, this, std::placeholders::_1));
        gimbal_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("euler_angles",
            10, std::bind(&OmniChassis::gimbal_callback, this, std::placeholders::_1));

        if (move_mode == "chassis")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move",
                10, std::bind(&OmniChassis::cha_callback, this, std::placeholders::_1));
        else if (move_mode == "absolute")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move",
                10, std::bind(&OmniChassis::abs_callback, this, std::placeholders::_1));
        else if (move_mode == "natural")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move",
                10, std::bind(&OmniChassis::nat_callback, this, std::placeholders::_1));
        else
            RCLCPP_ERROR(this->get_logger(), "Invalid move_mode: %s", move_mode.c_str());

        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&OmniChassis::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "Chassis mode set to %s", move_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "OmniChassis initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<device_interface::msg::MotorState>::SharedPtr motor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_; // ahrs feedback on gimbal
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::unique_ptr<OmniKinematics> kine;

    float gimbal_yaw_pos = 0.0; // The yaw position of the gimbal against the ground, in radians.
    float motor_yaw_pos = 0.0; // The yaw position of the gimbal against the chassis, in radians.

    void cha_callback(const behavior_interface::msg::Move::SharedPtr cha_msg)
    {
        kine->chassis_decompo(cha_msg);
    }

    void abs_callback(const behavior_interface::msg::Move::SharedPtr abs_msg)
    {
        kine->absolute_decompo(abs_msg, gimbal_yaw_pos, motor_yaw_pos);
    }

    void nat_callback(const behavior_interface::msg::Move::SharedPtr nat_msg)
    {
        kine->natural_decompo(nat_msg, motor_yaw_pos);
    }

    void motor_callback(const device_interface::msg::MotorState::SharedPtr motor_msg)
    {
        // update motor_yaw_pos
        int count = motor_msg->motor_id.size();
        for (int i = 0; i < count; i++) // look for the yaw motor
        {
            if (motor_msg->motor_id[i] == "YAW")
            {
                motor_yaw_pos = motor_msg->present_pos[i];
                break;
            }
        }
    }

    void gimbal_callback(const geometry_msgs::msg::Vector3::SharedPtr gimbal_msg)
    {
        gimbal_yaw_pos = gimbal_msg->z;
    }

    void pub_callback()
    {
        motor_pub_->publish(kine->get_motor_goal());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<OmniChassis>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}