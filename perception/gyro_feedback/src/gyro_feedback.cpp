#include "rclcpp/rclcpp.hpp"
#include "gyro_feedback/gyro_driver.h"

#include "gyro_interface/srv/gimbal_position.hpp"

class GyroFeedback : public rclcpp::Node
{
private:
    GyroDriver* gyro_driver_;
    rclcpp::Service<gyro_interface::srv::GimbalPosition>::SharedPtr gimbal_srv_;

    void gimbal_callback(const gyro_interface::srv::GimbalPosition::Request::SharedPtr request,
                         gyro_interface::srv::GimbalPosition::Response::SharedPtr response)
    {
    }

public:
    GyroFeedback() : Node("gyro_feedback")
    {
        gimbal_srv_ = this->create_service<gyro_interface::srv::GimbalPosition>("gimbal_position", std::bind(&GyroFeedback::gimbal_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GyroFeedback>());

    rclcpp::shutdown();
    return 0;
}