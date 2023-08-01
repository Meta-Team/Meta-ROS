#include "rclcpp/rclcpp.hpp"
#include "../include/dm_motor_controller/can_driver.h"
#include "dm_motor_interface/msg/mit_data.hpp"
#include "dm_motor_interface/msg/vel_data.hpp"
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
        subscription_vel_ = this->create_subscription<dm_motor_interface::msg::VelData>(
            "vel_data", 10, [this](const dm_motor_interface::msg::VelData::SharedPtr vel_data) {
                this->vel_data_callback(vel_data);
            });

        // this->declare_parameter("motor_name","motor_mode","motor_mid","motor_sid","motor_mitKp","motor_mitKi","motor_v_max","motor_p_max","motor_t_max","motor_initial_encoder_angle","motor_kp_min","motor_kp_max","motor_kd_min","motor_kd_max","start_cmd","stop_cmd","save_zero_cmd","clear_error_cmd")
    }

private:
    rclcpp::Subscription<dm_motor_interface::msg::MITData>::SharedPtr subscription_mit_;
    rclcpp::Subscription<dm_motor_interface::msg::VelData>::SharedPtr subscription_vel_;
    DmMotorDriver dm_motor_driver;
    // int motor_count;

    void mit_data_callback(const dm_motor_interface::msg::MITData::SharedPtr mit_data)
    {
        DmMotorCFG::MotorName motor_name = (DmMotorCFG::MotorName)mit_data->name;
        dm_motor_driver.set_mode(motor_name, MIT_MODE);
        float goal_pos = mit_data->p_des;
        float goal_vel = mit_data->v_des;
        dm_motor_driver.set_position(motor_name, goal_pos);
        dm_motor_driver.set_velocity(motor_name, goal_vel);
    }

    void vel_data_callback(const dm_motor_interface::msg::VelData::SharedPtr vel_data)
    {
        DmMotorCFG::MotorName motor_name = (DmMotorCFG::MotorName)vel_data->name;
        dm_motor_driver.set_mode(motor_name, VEL_MODE);
        float goal_vel = vel_data->v_des;
        dm_motor_driver.set_velocity(motor_name, goal_vel);
    }
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanMotorController>());
    rclcpp::shutdown();
    return 0;
}