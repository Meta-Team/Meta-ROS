#include "rclcpp/rclcpp.hpp"
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#include "dm_controller/can_driver.hpp"
#include "dm_controller/dm_driver.h"

#include "device_interface/msg/motor_goal.hpp"
#include "device_interface/msg/motor_state.hpp"

#define PUB_R 20

class DmController : public rclcpp::Node
{
public:
    DmController() : Node("dm_controller")
    {
        motor_init();
        goal_sub_ = this->create_subscription<device_interface::msg::MotorGoal>(
            "motor_goal", 10, [this](device_interface::msg::MotorGoal::SharedPtr msg){
                this->goal_callback(msg);
            });
        state_pub_ = this->create_publisher<device_interface::msg::MotorState>("motor_state", 10);
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(PUB_R), [this](){
            this->state_callback();
        });

        RCLCPP_INFO(this->get_logger(), "DmController initialized");
    }

    ~DmController()
    {
        for (auto& [_, driver] : dm_driver_)
        {
            driver->turn_off();
        }
    }

private:
    int dm_motor_count = 0;
    rclcpp::Subscription<device_interface::msg::MotorGoal>::SharedPtr goal_sub_;
    rclcpp::Publisher<device_interface::msg::MotorState>::SharedPtr state_pub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::unordered_map<std::string, std::unique_ptr<DmDriver>> dm_driver_;
    std::vector<double> kps{}, kis{};

    void motor_init()
    {
        int motor_count = this->declare_parameter("motor.count", 0);

        std::vector<std::string> motor_brands{};
        motor_brands = this->declare_parameter("motor.brands", motor_brands);
        std::vector<std::string> motor_rids{};
        motor_rids = this->declare_parameter("motor.rids", motor_rids);
        std::vector<int64_t> motor_hids{};
        motor_hids = this->declare_parameter("motor.hids", motor_hids);
        std::vector<std::string> motor_types{};
        motor_types= this->declare_parameter("motor.types", motor_types);
        std::vector<std::string> motor_ports{};
        motor_ports = this->declare_parameter("motor.ports", motor_ports);
        kps = this->declare_parameter("motor.p2v.kps", kps);
        kis = this->declare_parameter("motor.p2v.kis", kis);

        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] != "DM") continue; // only create drivers for DM motors

            dm_motor_count++;
            std::string rid = motor_rids[i];
            int hid = motor_hids[i];
            std::string port = motor_ports[i];

            if (motor_types[i] == "VEL")
            {
                dm_driver_[rid] = std::make_unique<DmVelDriver>(rid, hid, port);
            }
            else if (motor_types[i] == "MIT")
            {
                dm_driver_[rid] = std::make_unique<DmMitDriver>(rid, hid, kps[i], kis[i], port);
            }
            else {
                RCLCPP_WARN(this->get_logger(), "Motor %s type %s not supported", rid.c_str(), motor_types[i].c_str());
            }
        }

        for (auto& [_, driver] : dm_driver_)
        {
            driver->turn_on();
        }
    }

    void goal_callback(device_interface::msg::MotorGoal::SharedPtr msg)
    {
        int count = msg->motor_id.size();
        for (int i = 0; i < count; i++)
        {
            std::string rid = msg->motor_id[i];
            float pos = msg->goal_pos[i];
            float vel = msg->goal_vel[i];

            // find corresponding driver
            auto iter = dm_driver_.find(rid);

            // set goal
            if (iter != dm_driver_.end())
            {
                auto& driver = iter->second;
                driver->set_position(pos);
                driver->set_velocity(vel);
            }
            else {
                // not found, may be a dji motor
                // RCLCPP_WARN(this->get_logger(), "Motor %d not found", rid);
            }
        }
    }

    void state_callback()
    {
        device_interface::msg::MotorState state_msg;
        for (auto& [_, driver] : dm_driver_)
        {
            auto [pos, vel, tor] = driver->get_state();
            state_msg.motor_id.push_back(driver->rid);
            state_msg.present_pos.push_back(pos);
            state_msg.present_vel.push_back(vel);
            state_msg.present_tor.push_back(tor);
        }
        state_pub_->publish(state_msg);
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