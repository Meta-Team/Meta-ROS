#include "rclcpp/rclcpp.hpp"

#include "motor_interface/msg/motor_goal.hpp"
#include "aiming_interface/msg/uni_aiming_demo.hpp"
#include "movement_interface/msg/chassis_move.hpp"

enum MotorId
{
    YAW = 7,
    PITCH = 8,
};

class Aiming : public rclcpp::Node
{
public:
    Aiming() : Node("aiming")
    {
        yaw_ratio = this->declare_parameter("gimbal.yaw_ratio", yaw_ratio);
        motor_pub_ = this->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);
        aim_sub_ = this->create_subscription<aiming_interface::msg::UniAimingDemo>("uni_aiming_demo",
            10, std::bind(&Aiming::aim_callback, this, std::placeholders::_1));
        chassis_sub_ = this->create_subscription<movement_interface::msg::ChassisMove>("chassis_move",
            10, std::bind(&Aiming::chassis_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<aiming_interface::msg::UniAimingDemo>::SharedPtr aim_sub_;
    rclcpp::Subscription<movement_interface::msg::ChassisMove>::SharedPtr chassis_sub_;
    rclcpp::Publisher<motor_interface::msg::MotorGoal>::SharedPtr motor_pub_;

    float chassis_omega = 0.0;
    double yaw_ratio = 1./4.;

    void aim_callback(const aiming_interface::msg::UniAimingDemo::SharedPtr msg)
    {
        motor_interface::msg::MotorGoal goal_msg{};
        // yaw
        goal_msg.motor_id.push_back("YAW");
        goal_msg.goal_vel.push_back(- yaw_ratio * chassis_omega);
        goal_msg.goal_pos.push_back(0.0);
        // pitch
        goal_msg.motor_id.push_back("PITCH");
        goal_msg.goal_vel.push_back(0.0);
        goal_msg.goal_pos.push_back(msg->pitch_pos);

        motor_pub_->publish(goal_msg);
    }

    void chassis_callback(const movement_interface::msg::ChassisMove::SharedPtr msg)
    {
        chassis_omega = msg->omega;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<Aiming>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}