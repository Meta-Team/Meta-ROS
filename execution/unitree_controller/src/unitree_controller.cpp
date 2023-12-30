#include "rclcpp/rclcpp.hpp"
#include "unitree_controller/unitree_driver.hpp"
#include <memory>

class UnitreeController : public rclcpp::Node
{
public:
    UnitreeController() : Node("unitree_controller")
    {
    }

private:
    std::unique_ptr<UnitreeDriver> unitree_driver_; 
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeController>());
    rclcpp::shutdown();
    return 0;
}