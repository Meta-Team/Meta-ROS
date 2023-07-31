#include "rclcpp/rclcpp.hpp"
#include "../include/dm_motor_controller/can_driver.h"
#include "dm_motor_interface/msg/mit_data.hpp"
#include "../include/dm_motor_controller/dm_motor_driver.h"

class CanMotorController : public rclcpp::Node
{
public:
    CanMotorController() : Node("can_motor_controller")
    {
        subscription_mit_ = this->create_subscription<dm_motor_interface::msg::MITData>(
            "mit_data", 10, [this](const dm_motor_interface::msg::MITData::SharedPtr mit_data) {
                this->mit_data_callback(mit_data);
            });
    }

private:
    rclcpp::Subscription<dm_motor_interface::msg::MITData>::SharedPtr subscription_mit_;

    void mit_data_callback(const dm_motor_interface::msg::MITData::SharedPtr mit_data)
    {
        DmMotorCFG::MotorName motor_name = (DmMotorCFG::MotorName)mit_data->name;
        float goal_pos = mit_data->p_des;
        float goal_vel = mit_data->v_des;
        DmMotorDriver::set_position(motor_name, goal_pos);
        DmMotorDriver::set_velocity(motor_name, goal_vel);
    }
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMotorController>());
    rclcpp::shutdown();
    return 0;
}