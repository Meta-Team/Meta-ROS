#include "rclcpp/rclcpp.hpp"
#include "behavior_interface/msg/aim.hpp"
#include "behavior_interface/msg/move.hpp"
#include "behavior_interface/msg/shoot.hpp"

using namespace behavior_interface::msg;

class AutoSentry : public rclcpp::Node
{
public:
    AutoSentry() : Node("auto_sentry")
    {
        this->aim_pub_ = this->create_publisher<Aim>("aim", 10);
        this->move_pub_ = this->create_publisher<Move>("move", 10);
        this->shoot_pub_ = this->create_publisher<Shoot>("shoot", 10);
    }

private:
    rclcpp::Publisher<Aim>::SharedPtr aim_pub_;
    rclcpp::Publisher<Move>::SharedPtr move_pub_;
    rclcpp::Publisher<Shoot>::SharedPtr shoot_pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<AutoSentry>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}