#include "rclcpp/rclcpp.hpp"
#include "gyro_feedback/gyro_driver.h"

#include "gyro_interface/srv/gimbal_position.hpp"
#include <rclcpp/timer.hpp>

class GyroFeedback : public rclcpp::Node
{
private:
    GyroDriver* gyro_driver_;
    rclcpp::Service<gyro_interface::srv::GimbalPosition>::SharedPtr gimbal_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    void gimbal_callback(const gyro_interface::srv::GimbalPosition::Request::SharedPtr /*request*/,
                         gyro_interface::srv::GimbalPosition::Response::SharedPtr response)
    {
        response->yaw = gyro_driver_->yaw;
        response->roll = gyro_driver_->roll;
        response->pitch = gyro_driver_->pitch;
    }

    void timer_callback()
    {
        gyro_driver_->get_frame();
        gyro_driver_->process_rx();
        
    }

public:
    GyroFeedback() : Node("gyro_feedback")
    {
        gimbal_srv_ = this->create_service<gyro_interface::srv::GimbalPosition>("gimbal_position",
            std::bind(&GyroFeedback::gimbal_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<GyroFeedback>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}