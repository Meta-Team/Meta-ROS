#include "rclcpp/rclcpp.hpp"

#include "mecanum_chassis/mecanum_kinematics.hpp"

#include "movement_interface/msg/natural_move.hpp"
#include "movement_interface/msg/absolute_move.hpp"

class MecanumChassis : public rclcpp::Node
{
public:
    MecanumChassis() : Node("MecanumChassis")
    {
        
    }

private:
    rclcpp::Subscription<movement_interface::msg::NaturalMove>::SharedPtr nat_sub_;
    rclcpp::Subscription<movement_interface::msg::AbsoluteMove>::SharedPtr abs_sub_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}