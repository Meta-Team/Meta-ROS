#include "rclcpp/rclcpp.hpp"
#include "dji_controller/dji_driver.h"

#include "motor_interface/msg/dji_goal.hpp"
#include "motor_interface/srv/motor_present.hpp"
#include <cmath>
#include <cstdint>
#include <linux/can.h>
#include <memory>
#include <vector>

class DjiController : public rclcpp::Node
{
public:
    DjiController() : Node("DjiController")
    {
        motor_init();
        goal_sub_ = this->create_subscription<motor_interface::msg::DjiGoal>(
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

        param_ev_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        p2v_kps_cb_ = param_ev_->add_parameter_callback("p2v_kps", std::bind(&DjiController::p2v_kps_callback, this, std::placeholders::_1));
        p2v_kis_cb_ = param_ev_->add_parameter_callback("p2v_kis", std::bind(&DjiController::p2v_kis_callback, this, std::placeholders::_1));
        p2v_kds_cb_ = param_ev_->add_parameter_callback("p2v_kds", std::bind(&DjiController::p2v_kds_callback, this, std::placeholders::_1));
        v2c_kps_cb_ = param_ev_->add_parameter_callback("v2c_kps", std::bind(&DjiController::v2c_kps_callback, this, std::placeholders::_1));
        v2c_kis_cb_ = param_ev_->add_parameter_callback("v2c_kis", std::bind(&DjiController::v2c_kis_callback, this, std::placeholders::_1));
        v2c_kds_cb_ = param_ev_->add_parameter_callback("v2c_kds", std::bind(&DjiController::v2c_kds_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<motor_interface::msg::DjiGoal>::SharedPtr goal_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_; // send control frame regularly
    rclcpp::TimerBase::SharedPtr feedback_timer_; // receive feedback frame regularly
    std::shared_ptr<rclcpp::ParameterEventHandler> param_ev_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> p2v_kps_cb_, p2v_kis_cb_, p2v_kds_cb_, v2c_kps_cb_, v2c_kis_cb_, v2c_kds_cb_;
    int dji_motor_count;
    std::unique_ptr<DjiDriver> driver_[8];
    std::vector<double> p2v_kps{}, p2v_kis{}, p2v_kds{};
    std::vector<double> v2c_kps{}, v2c_kis{}, v2c_kds{};

    void control_timer_callback()
    {
        for (int i = 0; i < dji_motor_count; i++)
        {
            driver_[i]->write_tx();
            DjiDriver::send_frame();
        }
    }

    void feedback_timer_callback()
    {
        for (int i = 0; i < dji_motor_count; i++)
        {
            DjiDriver::get_frame();
            driver_[i]->process_rx();
            // rclcpp::sleep_for(std::chrono::milliseconds(1)); // sleep for 1 ms
        }
    }
    
    void goal_callback(const motor_interface::msg::DjiGoal::SharedPtr msg)
    {
        for (int i = 0; i < dji_motor_count; i++)
        {
            driver_[i]->set_goal(msg->goal_pos[i], msg->goal_vel[i]);
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

        // initialize the drivers
        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] != 0) continue; // only create drivers for DJI motors
            
            dji_motor_count++;
            MotorType type = static_cast<MotorType>(motor_types[i]);
            int id = motor_ids[i];
            driver_[i] = std::make_unique<DjiDriver>(id, type);
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
        
        for (int i = 0; i < dji_motor_count; i++)
        {
            driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
            driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
        }
    }

    // pid params callback
    void p2v_kds_callback(const rclcpp::Parameter & p)
    {
        p2v_kds.clear();
        p2v_kds = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
    }
    void v2c_kds_callback(const rclcpp::Parameter & p)
    {
        v2c_kds.clear();
        v2c_kds = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
    }
    void p2v_kis_callback(const rclcpp::Parameter & p)
    {
        p2v_kis.clear();
        p2v_kis = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
    }
    void v2c_kis_callback(const rclcpp::Parameter & p)
    {
        v2c_kis.clear();
        v2c_kis = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
    }
    void p2v_kps_callback(const rclcpp::Parameter & p)
    {
        p2v_kps.clear();
        p2v_kps = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_p2v_pid(p2v_kps[i], p2v_kis[i], p2v_kds[i]);
    }
    void v2c_kps_callback(const rclcpp::Parameter & p)
    {
        v2c_kps.clear();
        v2c_kps = p.as_double_array();
        for (int i = 0; i < dji_motor_count; i++) driver_[i]->set_v2c_pid(v2c_kps[i], v2c_kis[i], v2c_kds[i]);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DjiController>());
    rclcpp::shutdown();
    return 0;
}