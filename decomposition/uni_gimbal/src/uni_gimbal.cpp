#include "rclcpp/rclcpp.hpp"
#include "uni_gimbal/uni_gimbal.h"

class UniGimbal: public rclcpp::Node
{
public:
    UniGimbal() : Node("UniGimbal")
    {

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UniGimbal>());
    rclcpp::shutdown();
    return 0;
}