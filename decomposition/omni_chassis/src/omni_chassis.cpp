#include "rclcpp/rclcpp.hpp"

#include "omni_chassis/omni_kinematics.hpp"
#include <motor_interface/srv/detail/motor_present__struct.hpp>

#define YAW 4

class OmniChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<movement_interface::msg::NaturalMove>::SharedPtr nat_sub_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Client<gyro_interface::srv::GimbalPosition>::SharedPtr gimbal_cli_;
    rclcpp::Client<motor_interface::srv::MotorPresent>::SharedPtr motor_cli_;

    void abs_callback(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg)
    {
        auto request_gimbal = std::make_shared<gyro_interface::srv::GimbalPosition::Request>();
        auto result_gimbal = gimbal_cli_->async_send_request(request_gimbal);
        float gimbal_yaw = result_gimbal.get()->yaw;

        auto request_motor = std::make_shared<motor_interface::srv::MotorPresent::Request>();
        request_motor->motor_id.clear();
        request_motor->motor_id.push_back(YAW);
        auto result_motor = motor_cli_->async_send_request(request_motor);
        float motor_pos = result_motor.get()->present_pos[YAW];

        motor_pub_->publish(OmniKinematics::absolute_decompo(abs_msg, gimbal_yaw + motor_pos));
    }

public:
    OmniChassis() : Node("OmniChassis")
    {
        abs_sub_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
            "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
                this->abs_callback(msg);
            });
            
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);

        gimbal_cli_ = this->create_client<gyro_interface::srv::GimbalPosition>("gimbal_position");
        gimbal_cli_->wait_for_service(std::chrono::seconds(2));

        motor_cli_ = this->create_client<motor_interface::srv::MotorPresent>("motor_present");
        motor_cli_->wait_for_service(std::chrono::seconds(2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OmniChassis>());
    rclcpp::shutdown();
    return 0;
}