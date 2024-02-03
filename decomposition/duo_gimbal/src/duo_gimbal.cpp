#include "rclcpp/rclcpp.hpp"

class DuoGimbal : public rclcpp::Node
{
public:
    DuoGimbal() : Node("duo_gimbal")
    {
    }

private:
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<DuoGimbal>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}