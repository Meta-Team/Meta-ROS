#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/teleop_key.hpp"
#include "movement_interface/msg/chassis_move.hpp"

class TeleopChassis : public rclcpp::Node
{
public:
    TeleopChassis() : Node("TeleopChassis")
    {
        cha_pub_ = this->create_publisher<movement_interface::msg::ChassisMove>(
            "chassis_move", 10);
        key_sub_ = this->create_subscription<operation_interface::msg::TeleopKey>(
            "motor_goal", 10, [this](const operation_interface::msg::TeleopKey::SharedPtr msg){
                key_callback(msg);
            });
    }

private:
    rclcpp::Subscription<operation_interface::msg::TeleopKey>::SharedPtr key_sub_;
    rclcpp::Publisher<movement_interface::msg::ChassisMove>::SharedPtr cha_pub_;
    movement_interface::msg::ChassisMove cha_msg_;

    void key_callback(const operation_interface::msg::TeleopKey::SharedPtr msg)
    {
        // clear goals
        cha_msg_.vel_x = 0;
        cha_msg_.vel_y = 0;
        cha_msg_.omega = 0;

        // set goals
        if (msg->a == true) cha_msg_.vel_x = 500;
        if (msg->d == true) cha_msg_.vel_x = -500;
        if (msg->w == true) cha_msg_.vel_y = 500;
        if (msg->s == true) cha_msg_.vel_y = -500;
        if (msg->q == true) cha_msg_.omega = 500;
        if (msg->e == true) cha_msg_.omega = -500;

        // publish goals
        cha_pub_->publish(cha_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopChassis>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}