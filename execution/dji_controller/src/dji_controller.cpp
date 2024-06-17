#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "device_interface/msg/motor_goal.hpp"
#include "device_interface/msg/motor_state.hpp"

#define PUB_R 10 // ms
#define CONTROL_R 3 // ms

class DjiController : public rclcpp::Node
{
public:
    DjiController() : Node("DjiController")
    {
        // initialize the motors
        motor_init();

        // create subscriptions and timers
        goal_sub_ = this->create_subscription<device_interface::msg::MotorGoal>(
            "motor_goal", 10, [this](const device_interface::msg::MotorGoal::SharedPtr msg){
                goal_callback(msg);
            });

        // create a timer for publishing motor state
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_R), [this](){
                pub_timer_callback();
            });
        state_pub_ = this->create_publisher<device_interface::msg::MotorState>("motor_state", 10);

        // complete initialization
        RCLCPP_INFO(this->get_logger(), "DjiController initialized");
    }

private:
    rclcpp::Subscription<device_interface::msg::MotorGoal>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_; // send control frame regularly
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::Publisher<device_interface::msg::MotorState>::SharedPtr state_pub_;
    int dji_motor_count;
    std::unordered_map<std::string, std::unique_ptr<DjiDriver>> drivers_; // std::unique_ptr<DjiDriver> drivers_[8];
    std::vector<double> p2v_kps{}, p2v_kis{}, p2v_kds{};
    std::vector<double> v2c_kps{}, v2c_kis{}, v2c_kds{};

    void goal_callback(const device_interface::msg::MotorGoal::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received motor goal");
        int count = msg->motor_id.size();
        for (int i = 0; i < count; i++)
        {
            std::string rid = msg->motor_id[i];
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
                // not found, may be a dm motor
                // RCLCPP_WARN(this->get_logger(), "Motor %d not found", rid);
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
            msg.motor_id.push_back(driver->rid);
            auto [pos, vel, cur] = driver->get_state();
            msg.present_pos.push_back(pos);
            msg.present_vel.push_back(vel);
            msg.present_tor.push_back(cur);
        }
        state_pub_->publish(msg);
    }

    void motor_init()
    {
        int motor_count = this->declare_parameter("motor.count", 0);

        std::vector<std::string> motor_brands{};
        motor_brands = this->declare_parameter("motor.brands", motor_brands);
        std::vector<std::string> motor_rids{};
        motor_rids = this->declare_parameter("motor.rids", motor_rids);
        std::vector<int64_t> motor_hids{};
        motor_hids = this->declare_parameter("motor.hids", motor_hids);
        std::vector<std::string> motor_ports{};
        motor_ports = this->declare_parameter("motor.ports", motor_ports);
        std::vector<std::string> motor_types{};
        motor_types = this->declare_parameter("motor.types", motor_types);
        std::vector<int64_t> motor_cali(motor_count, 0); // default to 0, in case it is not configured
        motor_cali = this->declare_parameter("motor.cali", motor_cali);
        
        p2v_kps = this->declare_parameter("motor.p2v.kps", p2v_kps);
        p2v_kis = this->declare_parameter("motor.p2v.kis", p2v_kis);
        p2v_kds = this->declare_parameter("motor.p2v.kds", p2v_kds);
        v2c_kps = this->declare_parameter("motor.v2c.kps", v2c_kps);
        v2c_kis = this->declare_parameter("motor.v2c.kis", v2c_kis);
        v2c_kds = this->declare_parameter("motor.v2c.kds", v2c_kds);

        // initialize the drivers
        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] != "DJI") continue; // only create drivers for DJI motors
            
            dji_motor_count++;
            std::string type = motor_types[i];
            std::string rid = motor_rids[i];
            std::string port = motor_ports[i];
            int hid = motor_hids[i];
            int cali = motor_cali[i];
            drivers_[rid] = std::make_unique<DjiDriver>(rid, hid, type, port, cali);
            drivers_[rid]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            drivers_[rid]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);

            RCLCPP_INFO(this->get_logger(), "Motor rid %s hid %d port %d initialized, with %f %f %f %f %f %f",
                drivers_[rid]->rid.c_str(), drivers_[rid]->hid, drivers_[rid]->get_port(),
                p2v_kps[i], p2v_kis[i], p2v_kds[i], v2c_kps[i], v2c_kis[i], v2c_kds[i]);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiController>());
    rclcpp::shutdown();
    return 0;
}