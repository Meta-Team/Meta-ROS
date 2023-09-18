#include "rclcpp/rclcpp.hpp"

#include "agv_chassis/agv_kinematics.hpp"

class AgvChassis : public rclcpp::Node
{
private:
    rclcpp::Subscription<movement_interface::msg::NaturalMove>::SharedPtr nat_sub_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;
    rclcpp::Client<gyro_interface::srv::ChassisPosition>::SharedPtr chassis_cli_;
    rclcpp::Client<gyro_interface::srv::GimbalPosition>::SharedPtr gimbal_cli_;

    void nat_callback(const movement_interface::msg::NaturalMove::SharedPtr nat_msg)
    {
        
        // motor_pub_->publish(AgvKinematics::natural_decompo(nat_msg));
    }

    void abs_callback(const movement_interface::msg::AbsoluteMove::SharedPtr abs_msg)
    {
        auto request = std::make_shared<gyro_interface::srv::ChassisPosition::Request>();
        auto result = chassis_cli_->async_send_request(request);
        float chassis_yaw = result.get()->yaw;
        motor_pub_->publish(AgvKinematics::absolute_decompo(abs_msg,chassis_yaw));
    }
public:
    AgvChassis() : Node("AgvChassis")
    {
        nat_sub_ = this->create_subscription<movement_interface::msg::NaturalMove>(
            "natural_move", 10, [this](const movement_interface::msg::NaturalMove::SharedPtr msg){
                this->nat_callback(msg);
            });
        abs_sub_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
            "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
                this->abs_callback(msg);
            });
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        chassis_cli_ = this->create_client<gyro_interface::srv::ChassisPosition>("chassis_position");
        gimbal_cli_ = this->create_client<gyro_interface::srv::GimbalPosition>("gimbal_position");
        chassis_cli_->wait_for_service(std::chrono::seconds(2));
        gimbal_cli_->wait_for_service(std::chrono::seconds(2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgvChassis>());
    rclcpp::shutdown();
    return 0;
}