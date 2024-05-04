#include "rclcpp/rclcpp.hpp"
#include "agv_chassis/agv_kinematics.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "motor_interface/msg/motor_state.hpp"
#include <cstdint>
#include <rclcpp/logging.hpp>
#include <vector>

namespace ph = std::placeholders;
using std::string, std::vector;

class AgvChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_sub_;
    rclcpp::Subscription<motor_interface::msg::MotorState>::SharedPtr motor_sub_;

    double motor_yaw_pos = 0.0; // The yaw position of the gimbal against the chassis, in radians.
    double gimbal_yaw_pos = 0.0; // The yaw position of the gimbal against the ground, in radians.

    void cha_callback(const behavior_interface::msg::Move::SharedPtr cha_msg)
    {
        auto tx_msg = AgvKinematics::chassis_decompo(cha_msg);
        motor_pub_->publish(tx_msg);
    }

    void abs_callback(const behavior_interface::msg::Move::SharedPtr abs_msg)
    {
        auto tx_msg = AgvKinematics::absolute_decompo(abs_msg, gimbal_yaw_pos, motor_yaw_pos);
        motor_pub_->publish(tx_msg);
    }

    void nat_callback(const behavior_interface::msg::Move::SharedPtr nat_msg)
    {
        auto tx_msg = AgvKinematics::natural_decompo(nat_msg, motor_yaw_pos);
        motor_pub_->publish(tx_msg);
    }

    void gimbal_callback(const geometry_msgs::msg::Vector3::SharedPtr gimbal_msg)
    {
        gimbal_yaw_pos = gimbal_msg->z;
    }
    
    void motor_callback(const motor_interface::msg::MotorState::SharedPtr motor_msg)
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

public:
    AgvChassis() : Node("AgvChassis")
    {
        string mode = "chassis";
        mode = this->declare_parameter("chassis.move_mode", mode);
        RCLCPP_INFO(this->get_logger(), "Chassis mode: %s", mode.c_str());

        get_offsets();

        // initialize subscriber
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        motor_sub_ = this->create_subscription<motor_interface::msg::MotorState>("motor_state", 10,
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

        RCLCPP_INFO(this->get_logger(), "Chassis mode set to %s", mode.c_str());
        RCLCPP_INFO(this->get_logger(), "AgvChassis initialized");
    }

    void get_offsets()
    {
        double lf = this->declare_parameter("chassis.offsets.LF", 0.0);
        double rf = this->declare_parameter("chassis.offsets.RF", 0.0);
        double lb = this->declare_parameter("chassis.offsets.LB", 0.0);
        double rb = this->declare_parameter("chassis.offsets.RB", 0.0);
        AgvKinematics::offsets["LF_D"] = lf;
        AgvKinematics::offsets["RF_D"] = rf;
        AgvKinematics::offsets["LB_D"] = lb;
        AgvKinematics::offsets["RB_D"] = rb;
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