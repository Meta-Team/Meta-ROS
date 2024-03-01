#include "rclcpp/rclcpp.hpp"

#include "operation_interface/msg/teleop_key.hpp"
#include "shooting_interface/msg/shoot.hpp"

class TeleopShooting : public rclcpp::Node
{
public:
    TeleopShooting() : Node("teleop_shooting")
    {
        shoot_msg.id = 0;
        shoot_pub_ = this->create_publisher<shooting_interface::msg::Shoot>("shoot", 10);
        op_sub_ = this->create_subscription<operation_interface::msg::TeleopKey>("teleop_key",
            10, std::bind(&TeleopShooting::op_callback, this, std::placeholders::_1));
        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
            std::bind(&TeleopShooting::send_timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "TeleopShooting initialized.");
    }

private:
    rclcpp::Publisher<shooting_interface::msg::Shoot>::SharedPtr shoot_pub_;
    rclcpp::Subscription<operation_interface::msg::TeleopKey>::SharedPtr op_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr recov_timer_;

    shooting_interface::msg::Shoot shoot_msg{};

    void op_callback(const operation_interface::msg::TeleopKey::SharedPtr op_msg)
    {
        if (op_msg->o == true)
            shoot_msg.fric_state = false;
        if (op_msg->p == true)
            shoot_msg.fric_state = true;

        if (op_msg->space == true)
        {
            // toggle feed state
            if (shoot_msg.feed_state == true) shoot_msg.feed_state = false;
            else shoot_msg.feed_state = true;
        }
    }

    void send_timer_callback()
    {
        shoot_pub_->publish(shoot_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<TeleopShooting>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}
