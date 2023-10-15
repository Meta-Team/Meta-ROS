#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"
#include "motor_interface/srv/motor_present.hpp"
#include <cmath>
#include <cstdint>
#include <linux/can.h>

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
        cli_ = this->create_client<motor_interface::srv::MotorPresent>("motor_present");
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr sub_;
    rclcpp::Client<motor_interface::srv::MotorPresent>::SharedPtr cli_;
    int motor_count;
    DjiDriver* driver_[8];
    can_frame tx_frame_;

    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        auto request = std::make_shared<motor_interface::srv::MotorPresent::Request>();
        for (int i = 0; i < 4; i++) request->motor_id.push_back(msg->motor_id[i]);
        auto result = cli_->async_send_request(request);
        for (int i = 0; i < 4; i++) // update 4 present pos and vel
        {
            driver_[msg->motor_id[i]]->update_pos(result.get()->present_pos[i]);
            driver_[msg->motor_id[i]]->update_vel(result.get()->present_vel[i]);
        }
        for (int i = 0; i < 4; i++) // set 4 goal current
        {
            float current;
            DjiDriver::set_current(current, driver_[msg->motor_id[i]]->vel2current(msg->goal_vel[i]));
            DjiDriver::set_current(current, driver_[msg->motor_id[i]]->pos2current(msg->goal_pos[i]));
            std::uint16_t current_data = DjiDriver::float_to_uint(current, -I_MAX, I_MAX, 16);

            int id = msg->motor_id[i];
            if (id > 4) id -= 4; // id 5 is the same as id 1, etc.
            tx_frame_.data[2*id-2] = current_data >> 8; // id 1 is at index 0, id 2 is at index 2, etc.
            tx_frame_.data[2*id-1] = current_data & 0xff; // id 1 is at index 1, id 2 is at index 3, etc.
        }
        tx_frame_.can_id = 0x200;
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