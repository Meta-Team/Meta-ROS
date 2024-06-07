#include "rclcpp/rclcpp.hpp"
#include "agv_chassis/agv_kinematics.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "device_interface/msg/motor_state.hpp"
#include <cstdint>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <vector>

#define PUB_RATE 10 // ms

namespace ph = std::placeholders;
using std::string;

class AgvChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<device_interface::msg::MotorState>::SharedPtr motor_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::unique_ptr<AgvKinematics> kine;

    double motor_yaw_pos = 0.0; // The yaw position of the gimbal against the chassis, in radians.
    double gimbal_yaw_pos = 0.0; // The yaw position of the gimbal against the ground, in radians.

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

    void gimbal_callback(const geometry_msgs::msg::Vector3::SharedPtr gimbal_msg)
    {
        gimbal_yaw_pos = gimbal_msg->z;
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

    void pub_callback()
    {
        motor_pub_->publish(kine->get_motor_goal());
    }

public:
    AgvChassis() : Node("AgvChassis")
    {
        string mode = "chassis";
        mode = this->declare_parameter("chassis.move_mode", mode);
        double cha_r = this->declare_parameter("chassis.chassis_radius", 0.3);
        double wheel_r = this->declare_parameter("chassis.wheel_radius", 0.05);
        double decel_ratio = this->declare_parameter("chassis.deceleration_ratio", 20.0);
        double n_offset = this->declare_parameter("north_offset", 0.0);
        double yaw_offset = this->declare_parameter("chassis.yaw_offset", 0.0);
        kine = std::make_unique<AgvKinematics>(wheel_r, cha_r, decel_ratio, n_offset, yaw_offset);

        get_offsets();

        // initialize subscriber
        motor_pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
        motor_sub_ = this->create_subscription<device_interface::msg::MotorState>("motor_state", 10,
            std::bind(&AgvChassis::motor_callback, this, ph::_1));
        gimbal_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("euler_angles", 10,
            std::bind(&AgvChassis::gimbal_callback, this, ph::_1));

        if (mode == "chassis")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::cha_callback, this, ph::_1));
        else if (mode == "natural")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::nat_callback, this, ph::_1));
        else if (mode == "absolute")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move", 10,
                std::bind(&AgvChassis::abs_callback, this, ph::_1));
        else
            RCLCPP_ERROR(this->get_logger(), "Invalid mode: %s", mode.c_str());

        // initialize timer
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_RATE),
            std::bind(&AgvChassis::pub_callback, this));

        RCLCPP_INFO(this->get_logger(), "Chassis mode set to %s", mode.c_str());
        RCLCPP_INFO(this->get_logger(), "AgvChassis initialized");
    }

    void get_offsets()
    {
        double lf = this->declare_parameter("chassis.offsets.LF", 0.0);
        double rf = this->declare_parameter("chassis.offsets.RF", 0.0);
        double lb = this->declare_parameter("chassis.offsets.LB", 0.0);
        double rb = this->declare_parameter("chassis.offsets.RB", 0.0);
        kine->set_offsets(lf, rf, lb, rb);
        RCLCPP_INFO(this->get_logger(), "Offsets: LF=%f, RF=%f, LB=%f, RB=%f", lf, rf, lb, rb);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AgvChassis>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}