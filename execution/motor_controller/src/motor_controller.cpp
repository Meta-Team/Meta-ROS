#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "motor_controller/motor_driver.h"
#include "motor_controller/dji_motor.h"
#include "motor_controller/unitree_motor.h"
#include "motor_controller/mi_motor.h"
#include "motor_controller/dm_motor.h"

#include "device_interface/msg/motor_goal.hpp"
#include "device_interface/msg/motor_state.hpp"

using std::unique_ptr;
using std::string;
using std::vector;
using std::unordered_map;

#define PUB_R 5 // ms

class MotorController : public rclcpp::Node
{
public:
    MotorController() : Node("MotorController")
    {
        // initialize the motors
        motor_init();

        // create subscriptions
        goal_sub_ = this->create_subscription<device_interface::msg::MotorGoal>(
            "motor_goal", 10, [this](const device_interface::msg::MotorGoal::SharedPtr msg){
                goal_callback(msg);
            });

        // create a timer for publishing motor state
        state_pub_ = this->create_publisher<device_interface::msg::MotorState>("motor_state", 10);
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_R), [this](){
                pub_timer_callback();
            });

        // complete initialization
        RCLCPP_INFO(this->get_logger(), "MotorController initialized");
    }

private:
    rclcpp::Subscription<device_interface::msg::MotorGoal>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::Publisher<device_interface::msg::MotorState>::SharedPtr state_pub_;
    unordered_map<string, unique_ptr<MotorDriver>> drivers_; // unique_ptr<DjiDriver> drivers_[8];
    vector<double> p2v_kps{}, p2v_kis{}, p2v_kds{};
    vector<double> v2t_kps{}, v2t_kis{}, v2t_kds{};

    void goal_callback(const device_interface::msg::MotorGoal::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received motor goal");
        int count = msg->motor_id.size();
        for (int i = 0; i < count; i++)
        {
            string rid = msg->motor_id[i];
            double pos = msg->goal_pos[i];
            double vel = msg->goal_vel[i];
            double cur = msg->goal_tor[i];

            // find corresponding driver
            auto iter = drivers_.find(rid);

            // set goal
            if (iter != drivers_.end())
            {
                auto& driver = iter->second;
                driver->set_goal(pos, vel, cur);
                // RCLCPP_INFO(this->get_logger(), "%s receive goal", rid.c_str());
            }
            else {
                // RCLCPP_WARN(this->get_logger(), "Motor %s not found", rid.c_str());
            }
        }
    }

    void pub_timer_callback()
    {
        // publish feedback
        static device_interface::msg::MotorState msg;

        msg.motor_id.clear();
        msg.present_pos.clear();
        msg.present_vel.clear();
        msg.present_tor.clear();

        msg.motor_id.reserve(drivers_.size());
        msg.present_pos.reserve(drivers_.size());
        msg.present_vel.reserve(drivers_.size());
        msg.present_tor.reserve(drivers_.size());

        for (auto& [_, driver] : drivers_)
        {
            msg.motor_id.push_back(driver->get_rid());
            auto [pos, vel, cur] = driver->get_state();
            msg.present_pos.push_back(pos);
            msg.present_vel.push_back(vel);
            msg.present_tor.push_back(cur);
        }
        state_pub_->publish(msg);
    }

    void motor_init()
    {
        vector<bool> motor_enables{};
        motor_enables = this->declare_parameter("motor.enables", motor_enables);
        int motor_count = motor_enables.size();

        vector<string> motor_brands{};
        motor_brands = this->declare_parameter("motor.brands", motor_brands);
        vector<string> motor_rids{};
        motor_rids = this->declare_parameter("motor.rids", motor_rids);
        vector<int64_t> motor_hids{};
        motor_hids = this->declare_parameter("motor.hids", motor_hids);
        vector<string> motor_ports{};
        motor_ports = this->declare_parameter("motor.ports", motor_ports);
        vector<string> motor_types{};
        motor_types = this->declare_parameter("motor.types", motor_types);
        vector<int64_t> motor_cali(motor_count, 0); // default to 0, in case it is not configured
        motor_cali = this->declare_parameter("motor.cali", motor_cali);
        
        p2v_kps = this->declare_parameter("motor.p2v.kps", p2v_kps);
        p2v_kis = this->declare_parameter("motor.p2v.kis", p2v_kis);
        p2v_kds = this->declare_parameter("motor.p2v.kds", p2v_kds);
        v2t_kps = this->declare_parameter("motor.v2t.kps", v2t_kps);
        v2t_kis = this->declare_parameter("motor.v2t.kis", v2t_kis);
        v2t_kds = this->declare_parameter("motor.v2t.kds", v2t_kds);

        // initialize the drivers
        for (int i = 0; i < motor_count; i++)
        {
            if (!motor_enables[i]) continue;

            auto& brand = motor_brands[i];
            auto& rid = motor_rids[i];
            auto& hid = motor_hids[i];
            auto& port = motor_ports[i];
            auto& type = motor_types[i];
            auto& cali = motor_cali[i];
            auto& p2v_kp = p2v_kps[i];
            auto& p2v_ki = p2v_kis[i];
            auto& p2v_kd = p2v_kds[i];
            auto& v2t_kp = v2t_kps[i];
            auto& v2t_ki = v2t_kis[i];
            auto& v2t_kd = v2t_kds[i];

            if (brand == "DJI")
                drivers_[rid] = std::make_unique<DjiMotor>(rid, hid, type, port, cali);
            else if (brand == "UT")
                drivers_[rid] = std::make_unique<UnitreeMotor>(rid, hid, type, port, cali);
            else if (brand == "MI")
                drivers_[rid] = std::make_unique<MiMotor>(rid, hid, type, port);
            else if (brand == "DM")
                drivers_[rid] = std::make_unique<DmMotor>(rid, hid, type, port);
            else
                RCLCPP_WARN(this->get_logger(), "Unknown motor brand %s", brand.c_str());

            drivers_[rid]->set_param(p2v_kp, p2v_ki, p2v_kd, v2t_kp, v2t_ki, v2t_kd);
            drivers_[rid]->print_info();
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