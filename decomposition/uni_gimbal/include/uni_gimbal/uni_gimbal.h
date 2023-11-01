#ifndef UNI_GIMBAL
#define UNI_GIMBAL

#include "rclcpp/rclcpp.hpp"
#include "aiming_interface/msg/uni_aiming.hpp" // get aiming goal
#include "gyro_interface/srv/gimbal_position.hpp" // get current gimbal yaw
#include "movement_interface/msg/natural_move.hpp" // get chassis omega
#include "movement_interface/msg/absolute_move.hpp" // get chassis omega
// #include "motor_interface/srv/motor_goal.hpp" // publish motor goal

#define P 0.1
#define I 0.0
#define D 0.0

class UniGimbal : public rclcpp::Node
{
private:
    rclcpp::Subscription<aiming_interface::msg::UniAiming>::SharedPtr aim_sub_;
    rclcpp::Client<gyro_interface::srv::GimbalPosition>::SharedPtr gyro_cli_;
    rclcpp::Subscription<movement_interface::msg::NaturalMove>::SharedPtr nat_cli_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_cli_;
    
    float error_sum;
    float error_last;
    float chassis_omega;
    int motor_id;

    float get_current_gimbal_yaw();

    float velocity_pid(float error);

    void aiming_callback(const aiming_interface::msg::UniAiming::SharedPtr msg);

public:
    UniGimbal();
};

#endif