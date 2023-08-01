#include "rclcpp/rclcpp.hpp"
#include <rclcpp/duration.hpp>

#include "../include/motor_controller/motor_driver.hpp"

#define DaMiao 0
#define DJI 1

class MotorController : public rclcpp::Node
{
public:
    int motor_count;
    MotorDriver* motor_drivers_[16];

    MotorController(): Node("MotorController")
    {
        motor_init();
    }

    ~MotorController()
    {
        for (int i = 0; i < motor_count; i++)
        {
            delete motor_drivers_[i];
        }
    }

    void motor_init()
    {
        this->declare_parameter("motor_count");
        this->get_parameter("motor_count", motor_count);

        std::vector<std::string> motor_names;
        std::vector<int> motor_brands;
        std::vector<int> motor_modes;
        std::vector<int> motor_types;
        this->declare_parameter("motor_names");
        this->declare_parameter("motor_brands");
        this->declare_parameter("motor_modes");
        this->declare_parameter("motor_types");
        this->get_parameter("motor_names", motor_names);
        this->get_parameter("motor_brands", motor_brands);
        this->get_parameter("motor_modes", motor_modes);
        this->get_parameter("motor_types", motor_types);

        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] == DaMiao)
            {
                if (motor_modes[i] == VEL_MODE) {
                    motor_drivers_[i] = new DmVelMotorDriver(i, motor_types[i]);
                }
                else if (motor_modes[i] == MIT_MODE) {
                    std::vector<float> kp, ki;
                    this->declare_parameter("mit_kp");
                    this->declare_parameter("mit_ki");
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