#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <motor_interface/srv/detail/motor_present__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/service.hpp>
#include <vector>

#include "motor_feedback/motor_driver.hpp"
#include "motor_feedback/dm_motor_driver.h"
#include "motor_feedback/dji_motor_driver.h"

#include "motor_interface/srv/motor_present.hpp"

CanDriver* MotorDriver::can_0 = new CanDriver(0);

class MotorFeedback : public rclcpp::Node
{
private:
    rclcpp::Service<motor_interface::srv::MotorPresent>::SharedPtr srv_;

    void srv_callback(const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                      motor_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        
    }

public:
    int motor_count;
    MotorDriver* motor_drivers_[16]; // not fully used, up to 16 motors

    MotorFeedback(): Node("MotorFeedback")
    {
        motor_init();
        srv_ = this->create_service<motor_interface::srv::MotorPresent>(
            "motor_present", [this](const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                                    motor_interface::srv::MotorPresent::Response::SharedPtr response){
                this->srv_callback(request, response);
            });
    }

    ~MotorFeedback()
    {
        for (int i = 0; i < motor_count; i++)
        {
            delete motor_drivers_[i];
        }
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);

        std::vector<std::string> motor_names;
        std::vector<int64_t> motor_brands;
        std::vector<int64_t> motor_types;
        motor_names = this->declare_parameter("motor_names", motor_names);
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        motor_types = this->declare_parameter("motor_types", motor_types);

        // create corresponding drivers
        // for (int i = 0; i < motor_count; i++)
        // {
        //     if (motor_brands[i] == DaMiao)
        //     {
        //         if (motor_modes[i] == VEL_MODE) {
        //             motor_drivers_[i] = new DmVelMotorDriver(i, motor_types[i]);
        //         }
        //         else if (motor_modes[i] == MIT_MODE) {
        //             std::vector<float> kp, ki;
        //             kp.clear();
        //             kp.push_back(this->declare_parameter<float>("mit_kp", 0.0));
        //             ki.clear();
        //             ki.push_back(this->declare_parameter<float>("mit_ki", 0.0));
        //             motor_drivers_[i] = new DmMitMotorDriver(i, motor_types[i], kp[i], ki[i]);
        //         }
        //     }
        //     else if (motor_brands[i] == DJI)
        //     {
        //         if (motor_modes[i] == VEL_MODE) {
        //             motor_drivers_[i] = new DjiPosMotorDriver(i, motor_types[i]);
        //         }
        //         else if (motor_modes[i] == POS_MODE) {
        //             motor_drivers_[i] = new DjiVelMotorDriver(i, motor_types[i]);
        //         }
        //     }
        // }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    return 0;
}