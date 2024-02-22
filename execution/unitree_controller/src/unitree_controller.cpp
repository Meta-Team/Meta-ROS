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
            std::string rid = msg->motor_id[i];
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
                // RCLCPP_INFO(this->get_logger(), "Motor %d set goal", rid);
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
        int motor_count = this->declare_parameter("motor.count", 0);

        std::vector<int64_t> motor_brands{};
        motor_brands = this->declare_parameter("motor.brands", motor_brands);
        std::vector<std::string> motor_rids{};
        motor_rids = this->declare_parameter("motor.rids", motor_rids);
        std::vector<int64_t> motor_hids{};
        motor_hids = this->declare_parameter("motor.hids", motor_hids);
        std::vector<int64_t> motor_types{};
        motor_types = this->declare_parameter("motor.types", motor_types);
        
        p2v_kps = this->declare_parameter("motor.p2v.kps", p2v_kps);
        p2v_kds = this->declare_parameter("motor.p2v.kds", p2v_kds);

        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] != 2) continue;

            unitree_motor_count++;
            std::string rid = motor_rids[i];
            int hid = motor_hids[i];
            drivers_.push_back(std::make_unique<UnitreeDriver>(rid, hid));
            drivers_.back()->set_pid(p2v_kps[i], p2v_kds[i]);
            RCLCPP_INFO(this->get_logger(), "Motor rid %s hid %d initialized with kp %f kd %f",
                rid.c_str(), hid, p2v_kps[i], p2v_kds[i]);
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeController>());
    rclcpp::shutdown();
    return 0;
}