#include "rclcpp/rclcpp.hpp"

#include "mecanum_chassis/mecanum_kinematics.hpp"


class MecanumChassis : public rclcpp::Node
{
public:
    MecanumChassis() : Node("MecanumChassis")
    {
        
    }

private:

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}