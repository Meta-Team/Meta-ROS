#include "motor_feedback/motor_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <vector>

#include "motor_feedback/motor_driver.hpp"
#include "motor_feedback/dm_motor_driver.h"
#include "motor_feedback/dji_motor_driver.h"

#include "motor_interface/srv/motor_present.hpp"

#define DaMiao 1
#define DJI 0

#define FEEDBACK_R 10 // ms

// init static members
can_frame MotorDriver::rx_frame;
std::unique_ptr<CanDriver> MotorDriver::can_0 = std::make_unique<CanDriver>(0);

class MotorFeedback : public rclcpp::Node
{
private:
    rclcpp::Service<motor_interface::srv::MotorPresent>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    int motor_count;
    // std::unique_ptr<MotorDriver> motor_drivers_[16]; // not fully used, up to 16 motors
    std::vector<std::unique_ptr<MotorDriver>> motor_drivers_;
    // std::unordered_map<int, std::unique_ptr<MotorDriver>> motor_drivers_;

    MotorFeedback(): Node("MotorFeedback")
    {
        motor_drivers_.clear();
        motor_init();
        // RCLCPP_INFO(this->get_logger(), "Motors initialized");
        srv_ = this->create_service<motor_interface::srv::MotorPresent>(
            "motor_present", [this](const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                                                           motor_interface::srv::MotorPresent::Response::SharedPtr response){
                this->srv_callback(request, response);
            });
        // RCLCPP_INFO(this->get_logger(), "Service created");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(FEEDBACK_R), [this](){
            this->timer_callback();
        });
        // RCLCPP_INFO(this->get_logger(), "Timer created");
        RCLCPP_INFO(this->get_logger(), "MotorFeedback initialization complete");
    }

    void srv_callback(const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                      motor_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        for (auto desired_id : request->motor_id)
        {
            // find corresponding driver
            auto iter = std::find_if(motor_drivers_.begin(), motor_drivers_.end(),
                [desired_id](const std::unique_ptr<MotorDriver>& driver){
                    return driver->motor_id == desired_id;
                });
            
            // set response
            if (iter != motor_drivers_.end())
            {
                auto driver = iter->get();
                response->present_pos.push_back(driver->present_data.position);
                response->present_vel.push_back(driver->present_data.velocity);
                response->present_tor.push_back(driver->present_data.torque);
            }
            else
            {
                // set response all to 0, so that the length of response is the same as request
                response->present_pos.push_back(0);
                response->present_vel.push_back(0);
                response->present_tor.push_back(0);
                RCLCPP_ERROR(this->get_logger(), "invalid motor id");
            }
        }
    }

    void timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "Timer callback start");
        MotorDriver::get_frame();
        for (auto& driver: motor_drivers_) driver->process_rx();
        // RCLCPP_INFO(this->get_logger(), "Timer callback end");
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);

        std::vector<int64_t> motor_ids;
        std::vector<int64_t> motor_brands;
        std::vector<int64_t> motor_types;
        motor_ids = this->declare_parameter("motor_ids", motor_ids);
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        motor_types = this->declare_parameter("motor_types", motor_types);
        // RCLCPP_INFO(this->get_logger(), "Parameters loaded");

        // create corresponding drivers
        for (int i = 0; i < motor_count; i++)
        {
            int id = motor_ids[i];
            if (motor_brands[i] == DaMiao)
            {
                // motor_drivers_[i] = std::make_unique<DmMotorDriver>(id);
                motor_drivers_.push_back(std::make_unique<DmMotorDriver>(id));
            }
            else if (motor_brands[i] == DJI)
            {
                // motor_drivers_[i] = std::make_unique<DjiMotorDriver>(id, (MotorType)motor_types[i]);
                motor_drivers_.push_back(std::make_unique<DjiMotorDriver>(id, (MotorType)motor_types[i]));
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "invalid motor brand");
            }
        }
        // RCLCPP_INFO(this->get_logger(), "Motor drivers created");
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