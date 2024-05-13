#include "rclcpp/rclcpp.hpp"

#include "mecanum_chassis/mecanum_kinematics.hpp"

#include "behavior_interface/msg/move.hpp"
#include "device_interface/msg/motor_state.hpp"
#include "device_interface/msg/motor_goal.hpp"

class MecanumChassis : public rclcpp::Node
{
public:
    MecanumChassis() : Node("MecanumChassis")
    {
        // get params
        move_mode = this->declare_parameter("move_mode", "chassis");
        MecanumKinematics::cha_wid = this->declare_parameter("chassis.chassis_width", MecanumKinematics::cha_wid);
        MecanumKinematics::cha_len = this->declare_parameter("chassis.chassis_length", MecanumKinematics::cha_len);
        MecanumKinematics::wheel_angle = this->declare_parameter("chassis.wheel_angle", MecanumKinematics::wheel_angle);
        MecanumKinematics::wheel_r = this->declare_parameter("chassis.wheel_radius", MecanumKinematics::wheel_r);
        MecanumKinematics::decel_ratio = this->declare_parameter("chassis.deceleration_ratio", MecanumKinematics::decel_ratio);
        MecanumKinematics::yaw_offset = this->declare_parameter("chassis.yaw_offset", MecanumKinematics::yaw_offset);

        MecanumKinematics::cha_param.kp = this->declare_parameter("chassis.pid.kp", MecanumKinematics::cha_param.kp);
        MecanumKinematics::cha_param.ki = this->declare_parameter("chassis.pid.ki", MecanumKinematics::cha_param.ki);
        MecanumKinematics::cha_param.kd = this->declare_parameter("chassis.pid.kd", MecanumKinematics::cha_param.kd);

        // init publisher and subscribers
        motor_pub_ = this->create_publisher<device_interface::msg::MotorGoal>("motor_goal", 10);
        motor_sub_ = this->create_subscription<device_interface::msg::MotorState>("motor_state",
            10, std::bind(&MecanumChassis::motor_callback, this, std::placeholders::_1));
        if (move_mode == "chassis")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move",
                10, std::bind(&MecanumChassis::cha_callback, this, std::placeholders::_1));
        else if (move_mode == "natural")
            move_sub_ = this->create_subscription<behavior_interface::msg::Move>("move",
                10, std::bind(&MecanumChassis::nat_callback, this, std::placeholders::_1));
        else
            RCLCPP_ERROR(this->get_logger(), "Invalid move_mode: %s", move_mode.c_str());

        RCLCPP_INFO(this->get_logger(), "Chassis mode set to %s", move_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "MecanumChassis initialized.");
    }

private:
    rclcpp::Subscription<behavior_interface::msg::Move>::SharedPtr move_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Subscription<device_interface::msg::MotorState>::SharedPtr motor_sub_;

    std::string move_mode;
    double motor_yaw_pos = 0.0; // The yaw position of the gimbal against the chassis, in radians.

    void cha_callback(const behavior_interface::msg::Move::SharedPtr cha_msg)
    {
        auto tx_msg = MecanumKinematics::chassis_decompo(cha_msg->vel_x, cha_msg->vel_y, cha_msg->omega);
        motor_pub_->publish(tx_msg);
    }

    void nat_callback(const behavior_interface::msg::Move::SharedPtr nat_msg)
    {
        auto tx_msg = MecanumKinematics::natural_decompo(nat_msg, motor_yaw_pos);
        motor_pub_->publish(tx_msg);
    }

    void motor_callback(const device_interface::msg::MotorState::SharedPtr motor_msg)
    {
        int count = motor_msg->motor_id.size();
        for (int i = 0; i < count; i++)
        {
            if (motor_msg->motor_id[i] == "YAW")
            {
                motor_yaw_pos = motor_msg->present_pos[i];
                break;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<MecanumChassis>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}