#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <motor_interface/msg/detail/motor_goal__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription.hpp>

#include "../include/motor_controller/motor_driver.hpp"
#include "../include/motor_controller/dm_motor_driver.h"
#include "../include/motor_controller/dji_motor_driver.h"

#include "motor_interface/msg/motor_goal.hpp"

#define DaMiao 0
#define DJI 1

CanDriver* MotorDriver::can_0 = new CanDriver(0);

class MotorController : public rclcpp::Node
{
private:
    rclcpp::Subscription<motor_interface::msg::MotorGoal>::SharedPtr sub_;

    void msg_callback(motor_interface::msg::MotorGoal msg)
    {
        for (int i = 0; i < motor_count; i++)
        {
            motor_drivers_[i]->set_velocity(msg.goal_vel[i]);
            motor_drivers_[i]->set_position(msg.goal_pos[i]);
        }
    }

public:
    int motor_count;
    MotorDriver* motor_drivers_[16]; // not fully used, up to 16 motors

    MotorController(): Node("MotorController")
    {
        motor_init();
        sub_ = this->create_subscription<motor_interface::msg::MotorGoal>(
            "motor_goal", 10, [this](motor_interface::msg::MotorGoal::SharedPtr msg){
                this->msg_callback(*msg);
            });
    }

    ~MotorController()
    {
        for (int i = 0; i < motor_count; i++)
        {
            motor_drivers_[i]->turn_off();
            delete motor_drivers_[i];
        }
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);

        std::vector<std::string> motor_names;
        std::vector<int64_t> motor_brands;
        std::vector<int64_t> motor_modes;
        std::vector<int64_t> motor_types;
        motor_names = this->declare_parameter("motor_names", motor_names);
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        motor_modes = this->declare_parameter("motor_modes", motor_modes);
        motor_types = this->declare_parameter("motor_types", motor_types);

        // create corresponding drivers
        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] == DaMiao)
            {
                if (motor_modes[i] == VEL_MODE) {
                    motor_drivers_[i] = new DmVelMotorDriver(i, motor_types[i]);
                }
                else if (motor_modes[i] == MIT_MODE) {
                    std::vector<float> kp, ki;
                    kp.clear();
                    kp.push_back(this->declare_parameter<float>("mit_kp", 0.0));
                    ki.clear();
                    ki.push_back(this->declare_parameter<float>("mit_ki", 0.0));
                    motor_drivers_[i] = new DmMitMotorDriver(i, motor_types[i], kp[i], ki[i]);
                }
            }
            else if (motor_brands[i] == DJI)
            {
                if (motor_modes[i] == VEL_MODE) {
                    motor_drivers_[i] = new DjiPosMotorDriver(i, motor_types[i]);
                }
                else if (motor_modes[i] == POS_MODE) {
                    motor_drivers_[i] = new DjiVelMotorDriver(i, motor_types[i]);
                }
            }
        }

        for (int i = 0; i < motor_count; i++)
        {
            motor_drivers_[i]->turn_on();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}