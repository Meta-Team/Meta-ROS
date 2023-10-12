#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"

class DjiController : public rclcpp::Node
{
public:
    DjiController() : Node("DjiController")
    {
        motor_init();
        sub_ = this->create_subscription<motor_interface::msg::DjiGoal>(
            "motor_goal", 10, [this](const motor_interface::msg::DjiGoal::SharedPtr msg){
                goal_callback(msg);
            });
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr sub_;
    int motor_count;
    DjiDriver* driver_[8];

    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);
        std::vector<int64_t> motor_modes;
        motor_modes = this->declare_parameter("motor_types", motor_modes);
        // for (int i = 0, i < motor_count, i++)
        // {
        //     if (motor_modes[i] == M3508)
        //     {
        //         driver_[i] = new Dji3508Driver(i);
        //     }
        //     else if (motor_modes[i] == M2006)
        //     {
        //         driver_[i] = new Dji2006Driver(i);
        //     }
        //     else if (motor_modes[i] == M6020)
        //     {
        //         driver_[i] = new Dji6020Driver(i);
        //     }
        //     driver_[i]->turn_on();
        // }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiController>());
    rclcpp::shutdown();
    return 0;
}