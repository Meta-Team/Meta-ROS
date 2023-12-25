#include "rclcpp/rclcpp.hpp"

#include "dm_controller/can_driver.hpp"
#include "dm_controller/dm_driver.h"

#include "motor_interface/msg/dm_goal.hpp"
#include <memory>

#define VEL_MODE 0
#define MIT_MODE 1

class DmController : public rclcpp::Node
{
public:
    DmController() : Node("dm_controller")
    {
        motor_init();
        sub_ = this->create_subscription<motor_interface::msg::DmGoal>(
            "dm_goal", 10, [this](motor_interface::msg::DmGoal::SharedPtr msg){
                this->msg_callback(*msg);
            });
    }

    ~DmController()
    {
        for (int i = 0; i < motor_count; i++)
        {
            dm_driver_[i]->turn_off();
        }
    }

private:
    int motor_count;
    rclcpp::Subscription<motor_interface::msg::DmGoal>::SharedPtr sub_;
    std::unique_ptr<DmDriver> dm_driver_[8];

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);
        std::vector<int64_t> motor_modes;
        motor_modes = this->declare_parameter("motor_types", motor_modes);

        for (int i = 0; i < motor_count; i++)
        {
            if (motor_modes[i] == VEL_MODE)
            {
                dm_driver_[i] = std::make_unique<DmVelDriver>(i);
            }
            else if (motor_modes[i] == MIT_MODE)
            {
                std::vector<float> kp, ki;
                kp.clear();
                kp.push_back(this->declare_parameter<float>("mit_kp", 0.0));
                ki.clear();
                ki.push_back(this->declare_parameter<float>("mit_ki", 0.0));
                dm_driver_[i] = std::make_unique<DmMitDriver>(i, kp[i], ki[i]);
            }
            dm_driver_[i]->turn_on();
        }
    }

    void msg_callback(motor_interface::msg::DmGoal msg)
    {
        int goal_count = msg.motor_id.size();
        for (int i = 0; i < goal_count; i++)
        {
            dm_driver_[msg.motor_id[i]]->set_velocity(msg.goal_vel[i]);
            dm_driver_[msg.motor_id[i]]->set_position(msg.goal_pos[i]);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}