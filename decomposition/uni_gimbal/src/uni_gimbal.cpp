#include "rclcpp/rclcpp.hpp"
#include "uni_gimbal/uni_gimbal.h"

UniGimbal::UniGimbal() : Node("uni_gimbal")
{
    aim_sub_ = this->create_subscription<aiming_interface::msg::UniAiming>(
        "uni_aiming", 10, [this](const aiming_interface::msg::UniAiming::SharedPtr msg){
            this->aiming_callback(msg);
        });

    nat_cli_ = this->create_subscription<movement_interface::msg::NaturalMove>(
        "natural_move", 10, [this](const movement_interface::msg::NaturalMove::SharedPtr msg){
            this->chassis_omega = msg->omega;
        });
    
    abs_cli_ = this->create_subscription<movement_interface::msg::AbsoluteMove>(
        "absolute_move", 10, [this](const movement_interface::msg::AbsoluteMove::SharedPtr msg){
            this->chassis_omega = msg->omega;
        });

    gyro_cli_ = this->create_client<gyro_interface::srv::GimbalPosition>("gimbal_position");
    gyro_cli_->wait_for_service(std::chrono::seconds(2));

    error_sum = 0.0;
    error_last = 0.0;

    motor_id = this->declare_parameter<int>("motor_id", 4);
}

float UniGimbal::get_current_gimbal_yaw()
{
    auto request = std::make_shared<gyro_interface::srv::GimbalPosition::Request>();
    auto result = gyro_cli_->async_send_request(request);
    return result.get()->yaw;
}

float UniGimbal::velocity_pid(float error)
{
    float error_diff = error - error_last;
    error_sum += error;
    error_last = error;
    return P * error + I * error_sum + D * error_diff;
    // when error is positive, goal is more counter-clockwise
    // thereor, velocity should be positive, which means gimbal rotates counter-clockwise
}

void UniGimbal::aiming_callback(const aiming_interface::msg::UniAiming::SharedPtr msg)
{
    float current_yaw = get_current_gimbal_yaw();
    float goal_yaw = msg->yaw_pos;
    float error = goal_yaw - current_yaw;
    float velocity = velocity_pid(error) - chassis_omega;
    // gimbal should rotate in reverse direction of chassis

    // motor_interface::msg::MotorGoal motor_goal;
    // motor_goal.motor_id.push_back(motor_id);
    // motor_goal.goal_vel.push_back(velocity);
    // motor_goal.goal_pos.push_back(0.0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UniGimbal>());
    rclcpp::shutdown();
    return 0;
}