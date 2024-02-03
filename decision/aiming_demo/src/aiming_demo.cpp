#include "rclcpp/rclcpp.hpp"

#include "aiming_interface/msg/uni_aiming_demo.hpp"
#include "operation_interface/msg/teleop_key.hpp"
#include <rclcpp/publisher.hpp>

class AimingDemo : public rclcpp::Node
{
public:
    AimingDemo() : Node("aiming_demo")
    {
    }

private:
    rclcpp::Publisher<aiming_interface::msg::UniAimingDemo>::SharedPtr aim_pub_;
    rclcpp::Subscription<operation_interface::msg::TeleopKey>::SharedPtr op_sub_;

    void op_callback(const operation_interface::msg::TeleopKey::SharedPtr msg)
    {
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