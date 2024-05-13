#include "rclcpp/rclcpp.hpp"

#include "omni_chassis/omni_kinematics.hpp"

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_state.hpp"
#include "device_interface/msg/motor_goal.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class OmniChassis : public rclcpp::Node
{
public:
    OmniChassis() : Node("OmniChassis")
    {
        // get params
        std::string move_mode = this->declare_parameter("move_mode", "chassis");
        OmniKinematics::cha_r = this->declare_parameter("chassis.chassis_radius", OmniKinematics::cha_r);
        OmniKinematics::wheel_r = this->declare_parameter("chassis.wheel_radius", OmniKinematics::wheel_r);
        OmniKinematics::decel_ratio = this->declare_parameter("chassis.deceleration_ratio", OmniKinematics::decel_ratio);
        OmniKinematics::n_offset = this->declare_parameter("north_offset", OmniKinematics::n_offset);
        OmniKinematics::yaw_offset = this->declare_parameter("chassis.yaw_offset", OmniKinematics::yaw_offset);

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

        RCLCPP_INFO(this->get_logger(), "Chassis mode set to %s", move_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "OmniChassis initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<device_interface::msg::MotorState>::SharedPtr motor_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_; // ahrs feedback on gimbal

    float gimbal_yaw_pos = 0.0; // The yaw position of the gimbal against the ground, in radians.
    float motor_yaw_pos = 0.0; // The yaw position of the gimbal against the chassis, in radians.

    void cha_callback(const behavior_interface::msg::Move::SharedPtr cha_msg)
    {
        auto tx_msg = OmniKinematics::chassis_decompo(cha_msg);
        motor_pub_->publish(tx_msg);
    }

    void abs_callback(const behavior_interface::msg::Move::SharedPtr abs_msg)
    {
        auto tx_msg = OmniKinematics::absolute_decompo(abs_msg, gimbal_yaw_pos, motor_yaw_pos);
        motor_pub_->publish(tx_msg);
    }

    void nat_callback(const behavior_interface::msg::Move::SharedPtr nat_msg)
    {
        auto tx_msg = OmniKinematics::natural_decompo(nat_msg, motor_yaw_pos);
        motor_pub_->publish(tx_msg);
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<OmniChassis>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}