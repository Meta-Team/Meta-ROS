#include "rclcpp/rclcpp.hpp"
#include "dbus_vehicle/dbus_interpreter.h"
#include <memory>

class DbusVehicle : public rclcpp::Node
{
public:
    DbusVehicle() : Node("dbus_vehicle")
    {
        RCLCPP_INFO(this->get_logger(), "DbusVehicle initialized.");
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_ = std::shared_ptr<DbusVehicle>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}