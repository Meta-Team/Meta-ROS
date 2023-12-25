#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"
#include "motor_interface/srv/motor_present.hpp"
#include <cmath>
#include <cstdint>
#include <linux/can.h>
#include <vector>

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
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(CONTROL_R), [this](){
                control_timer_callback();
            });
        feedback_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(FEEDBACK_R), [this](){
                feedback_timer_callback();
            });
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr sub_;
    rclcpp::Client<motor_interface::srv::MotorPresent>::SharedPtr cli_;
    rclcpp::TimerBase::SharedPtr control_timer_; // send control frame regularly
    rclcpp::TimerBase::SharedPtr feedback_timer_; // receive feedback frame regularly
    int motor_count;
    DjiDriver* driver_[8];
    std::vector<double> p2v_kps, p2v_kis, p2v_kds;
    std::vector<double> v2c_kps, v2c_kis, v2c_kds;

    void control_timer_callback()
    {
        for (int i = 0; i < motor_count; i++)
        {
            driver_[i]->write_frame();
            DjiDriver::send_frame();
        }
    }

    void feedback_timer_callback()
    {
        for (int i = 0; i < motor_count; i++)
        {
            DjiDriver::get_frame();
            driver_[i]->process_rx();
            rclcpp::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
        }
    }
    
    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        update_pid();
        for (int i = 0; i < motor_count; i++)
        {
            driver_[i]->set_goal(msg->goal_pos[i], msg->goal_vel[i]);
        }
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);
        std::vector<int64_t> motor_ids;
        motor_ids = this->declare_parameter("motor_ids", motor_ids);
        std::vector<int64_t> motor_modes;
        motor_modes = this->declare_parameter("motor_types", motor_modes);

        // initialize the drivers
        for (int i = 0; i < motor_count; i++)
        {
            MotorType type = static_cast<MotorType>(motor_modes[i]);
            int id = motor_ids[i];
            driver_[i] = new DjiDriver(id, type);
        }

        // initialize the pid params
        update_pid();
    }

    void update_pid()
    {
        p2v_kps = this->declare_parameter("p2v_kps", p2v_kps);
        p2v_kis = this->declare_parameter("p2v_kis", p2v_kis);
        p2v_kds = this->declare_parameter("p2v_kds", p2v_kds);

        v2c_kps = this->declare_parameter("v2c_kps", v2c_kps);
        v2c_kis = this->declare_parameter("v2c_kis", v2c_kis);
        v2c_kds = this->declare_parameter("v2c_kds", v2c_kds);
        
        for (int i = 0; i < motor_count; i++)
        {
            driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
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