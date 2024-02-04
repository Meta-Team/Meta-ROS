#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/teleop_key.hpp"
#include "movement_interface/msg/chassis_move.hpp"
#include <rclcpp/logging.hpp>

class TeleopChassis : public rclcpp::Node
{
public:
    TeleopChassis() : Node("TeleopChassis")
    {
        cha_pub_ = this->create_publisher<movement_interface::msg::ChassisMove>(
            "chassis_move", 10);
        key_sub_ = this->create_subscription<operation_interface::msg::TeleopKey>(
            "teleop_key", 10, [this](const operation_interface::msg::TeleopKey::SharedPtr key_msg_) { 
                key_callback(key_msg_);
            });
        send_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), [this](){
                send_callback();
            });
        recov_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200), [this](){
                recov_callback();
            });
        RCLCPP_INFO(this->get_logger(), "TeleopChassis initialized.");
    }

private:
    rclcpp::Subscription<operation_interface::msg::TeleopKey>::SharedPtr key_sub_;
    rclcpp::Publisher<movement_interface::msg::ChassisMove>::SharedPtr cha_pub_;
    movement_interface::msg::ChassisMove cha_msg_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr recov_timer_;

    movement_interface::msg::ChassisMove move_msg{};
    bool to_stop;

    void key_callback(const operation_interface::msg::TeleopKey::SharedPtr msg)
    {
        to_stop = false;
        // set goals
        if (msg->a == true) cha_msg_.vel_x = +500;
        if (msg->d == true) cha_msg_.vel_x = -500;
        if (msg->w == true) cha_msg_.vel_y = +500;
        if (msg->s == true) cha_msg_.vel_y = -500;
        if (msg->q == true) cha_msg_.omega = +100;
        if (msg->e == true) cha_msg_.omega = -100;
    }

    void send_callback()
    {
        cha_pub_->publish(cha_msg_);
    }

    void recov_callback()
    {
        if (to_stop == true) stop();
        to_stop = true;
    }

    void stop()
    {
        cha_msg_.vel_x = 0;
        cha_msg_.vel_y = 0;
        cha_msg_.omega = 0;
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