#include "rclcpp/rclcpp.hpp"

#include "aiming_interface/msg/uni_aiming_demo.hpp"
#include "operation_interface/msg/teleop_key.hpp"

#define PUB_R 20 // ms, publish rate

class AimingDemo : public rclcpp::Node
{
public:
    AimingDemo() : Node("aiming_demo")
    {
        aim_msg.pitch_pos = 0.0; // MY_TODO: to be tuned
        aim_pub_ = this->create_publisher<aiming_interface::msg::UniAimingDemo>("uni_aiming_demo", 10);
        op_sub_ = this->create_subscription<operation_interface::msg::TeleopKey>("teleop_key",
            10, std::bind(&AimingDemo::op_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_R),
            std::bind(&AimingDemo::timer_callback, this));
    }

private:
    rclcpp::Publisher<aiming_interface::msg::UniAimingDemo>::SharedPtr aim_pub_;
    rclcpp::Subscription<operation_interface::msg::TeleopKey>::SharedPtr op_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    aiming_interface::msg::UniAimingDemo aim_msg{};

    void op_callback(const operation_interface::msg::TeleopKey::SharedPtr msg)
    {
        if (msg->i == true) aim_msg.pitch_pos += 0.1;
        if (msg->j == true) aim_msg.pitch_pos -= 0.1;
    }

    void timer_callback()
    {
        aim_pub_->publish(aim_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AimingDemo>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}