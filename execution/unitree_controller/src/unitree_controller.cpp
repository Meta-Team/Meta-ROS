#include "rclcpp/rclcpp.hpp"
#include "unitree_controller/unitree_driver.hpp"
#include <memory>
#include <motor_interface/msg/detail/motor_goal__struct.hpp>
#include <rclcpp/subscription.hpp>
#include <vector>

#include "motor_interface/msg/motor_goal.hpp"

class UnitreeController : public rclcpp::Node
{
public:
    UnitreeController() : Node("unitree_controller")
    {
        motor_init();
        goal_sub_ = this->create_subscription<motor_interface::msg::MotorGoal>(
            "motor_goal", 10, [this](const motor_interface::msg::MotorGoal::SharedPtr msg){
                goal_callback(msg);
            });

        RCLCPP_INFO(this->get_logger(), "UnitreeController initialized");
    }

private:
    std::vector<std::unique_ptr<UnitreeDriver>> drivers_;
    rclcpp::Subscription<motor_interface::msg::MotorGoal>::SharedPtr goal_sub_;
    std::vector<double> p2v_kps{}, p2v_kds{};
    int unitree_motor_count;

    void goal_callback(const motor_interface::msg::MotorGoal::SharedPtr msg)
    {
        int count = msg->motor_id.size();
        for (int i = 0; i < count; i++)
        {
            int rid = msg->motor_id[i];
            float pos = msg->goal_pos[i];
            float vel = msg->goal_vel[i];

            // find corresponding driver
            auto iter = std::find_if(drivers_.begin(), drivers_.end(),
                [rid](const std::unique_ptr<UnitreeDriver>& driver){
                    return driver->rid == rid;
                });

            // set goal
            if (iter != drivers_.end())
            {
                auto driver = iter->get();
                driver->set_goal(pos, vel);
            }
            else {
                // not found, may be a dm motor
                // RCLCPP_WARN(this->get_logger(), "Motor %d not found", rid);
            }
        }
    }

    void motor_init()
    {
        int motor_count = this->declare_parameter("motor_count", 0);

        std::vector<int64_t> motor_brands {};
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        std::vector<int64_t> motor_ids {};
        motor_ids = this->declare_parameter("motor_ids", motor_ids);
        std::vector<int64_t> motor_types {};
        motor_types= this->declare_parameter("motor_types", motor_types);

        p2v_kps = this->declare_parameter("p2v_kps", p2v_kps);
        p2v_kds = this->declare_parameter("p2v_kds", p2v_kds);

        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] != 2) continue;

            unitree_motor_count++;
            int rid = motor_ids[i];
            drivers_.push_back(std::make_unique<UnitreeDriver>(rid));
            drivers_.back()->set_pid(p2v_kps[i], p2v_kds[i]);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeController>());
    rclcpp::shutdown();
    return 0;
}