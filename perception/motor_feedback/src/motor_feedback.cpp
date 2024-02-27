#include "motor_feedback/motor_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <vector>

#include "motor_feedback/motor_driver.hpp"
#include "motor_feedback/dm_motor_driver.h"
#include "motor_feedback/dji_motor_driver.h"

#include "motor_interface/srv/motor_state.hpp"
#include "motor_interface/msg/motor_state.hpp"

#define DaMiao 1
#define DJI 0

#define FEEDBACK_R 10 // ms

// init static members
can_frame MotorDriver::rx_frame;
std::unique_ptr<CanDriver> MotorDriver::can_0 = std::make_unique<CanDriver>(0);

class MotorFeedback : public rclcpp::Node
{
private:
    // publish and create service for motor state
    // either can be removed if not needed
    rclcpp::Publisher<motor_interface::msg::MotorState>::SharedPtr pub_;
    rclcpp::Service<motor_interface::srv::MotorState>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer;

public:
    int motor_count;
    std::vector<std::unique_ptr<MotorDriver>> motor_drivers_;

    MotorFeedback(): Node("MotorFeedback")
    {
        motor_drivers_.clear();
        motor_init();
        srv_ = this->create_service<motor_interface::srv::MotorState>(
            "motor_present", [this](const motor_interface::srv::MotorState::Request::SharedPtr request,
                                                           motor_interface::srv::MotorState::Response::SharedPtr response){
                this->srv_callback(request, response);
            });
        pub_ = this->create_publisher<motor_interface::msg::MotorState>("motor_present", 10);
        update_timer_ = this->create_wall_timer(std::chrono::milliseconds(FEEDBACK_R), [this](){
            this->update_timer_callback();
        });
        publish_timer = this->create_wall_timer(std::chrono::milliseconds(100), [this](){
            this->publish_timer_callback();
        });
        RCLCPP_INFO(this->get_logger(), "MotorFeedback initialization complete");
    }

    void srv_callback(const motor_interface::srv::MotorState::Request::SharedPtr request,
                      motor_interface::srv::MotorState::Response::SharedPtr response)
    {
        for (std::string desired_rid : request->motor_id)
        {
            // find corresponding driver
            auto iter = std::find_if(motor_drivers_.begin(), motor_drivers_.end(),
                [desired_rid](const std::unique_ptr<MotorDriver>& driver){
                    return driver->rid == desired_rid;
                });
            
            // set response
            if (iter != motor_drivers_.end())
            {
                auto driver = iter->get();
                response->present_pos.push_back(driver->present_data.position);
                response->present_vel.push_back(driver->present_data.velocity);
                response->present_tor.push_back(driver->present_data.torque);
            }
            else {
                // set response all to 0, so that the length of response is the same as request
                response->present_pos.push_back(0);
                response->present_vel.push_back(0);
                response->present_tor.push_back(0);
                RCLCPP_WARN(this->get_logger(), "Motor %s not found", desired_rid.c_str());
            }
        }
    }

    void publish_timer_callback()
    {
        motor_interface::msg::MotorState msg;
        msg.motor_id.clear();
        msg.present_pos.clear();
        msg.present_vel.clear();
        msg.present_tor.clear();
        for (auto& driver: motor_drivers_)
        {
            msg.motor_id.push_back(driver->rid);
            msg.present_pos.push_back(driver->present_data.position);
            msg.present_vel.push_back(driver->present_data.velocity);
            msg.present_tor.push_back(driver->present_data.torque);
        }
        pub_->publish(msg);
    }

    void update_timer_callback()
    {
        MotorDriver::get_frame();
        for (auto& driver: motor_drivers_) driver->process_rx();
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

        // create corresponding drivers
        for (int i = 0; i < motor_count; i++)
        {
            int hid = motor_hids[i];
            std::string rid = motor_rids[i];
            if (motor_brands[i] == DaMiao)
            {
                motor_drivers_.push_back(std::make_unique<DmMotorDriver>(rid, hid));
            }
            else if (motor_brands[i] == DJI)
            {
                motor_drivers_.push_back(std::make_unique<DjiMotorDriver>(rid, hid, (MotorType)motor_types[i]));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid motor brand %d", static_cast<int>(motor_brands[i]));
            }
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorFeedback>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}