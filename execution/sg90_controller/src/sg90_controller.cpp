#include "sg90_controller/gpio_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logging.hpp>

#define PI 3.14159

class SG90Driver
{
public:
    SG90Driver()
    {
        gpio_driver_->init_port(203);
    }

    float position_to_duty_time(float position) // rad to ms
    {
        return 0.5 + (position * 2.0) / (PI / 2.0);
    }

    void one_loop(int duty_time)  // last 20ms
    {
        gpio_driver_->set_high(203);
        rclcpp::sleep_for(std::chrono::milliseconds(duty_time));
        gpio_driver_->set_low(203);
        rclcpp::sleep_for(std::chrono::milliseconds(20 - duty_time));
    }

    ~SG90Driver()
    {
        gpio_driver_->end_port(203);
    }

private:
    GPIODriver* gpio_driver_;

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rclcpp init successfully");

    SG90Driver sg90_driver;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sg90_controller start running");

    while (true) {
  
        for (int i = 0; i < 50; i++) {
            sg90_driver.one_loop(sg90_driver.position_to_duty_time(0));
        }
        for (int i = 0; i < 50; i++) {
            sg90_driver.one_loop(sg90_driver.position_to_duty_time(PI));
        }
    }

    rclcpp::shutdown();
    return 0;
}