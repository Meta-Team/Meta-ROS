#include "motor_feedback/motor_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <rclcpp/timer.hpp>
#include <vector>

#include "motor_feedback/motor_driver.hpp"
#include "motor_feedback/dm_motor_driver.h"
#include "motor_feedback/dji_motor_driver.h"

#include "motor_interface/srv/motor_present.hpp"

#define DaMiao 0
#define DJI 1

#define DT 10 // ms

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
    std::unique_ptr<MotorDriver> motor_drivers_[16]; // not fully used, up to 16 motors

    MotorFeedback(): Node("MotorFeedback")
    {
        motor_init();
        srv_ = this->create_service<motor_interface::srv::MotorPresent>(
            "motor_present", [this](const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                                    motor_interface::srv::MotorPresent::Response::SharedPtr response){
                this->srv_callback(request, response);
            });
        timer_ = this->create_wall_timer(std::chrono::milliseconds(DT), [this](){
            this->timer_callback();
        });
    }

    void srv_callback(const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                      motor_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        int feedback_count = request->motor_id.size();
        for (int i = 0; i < feedback_count; i++)
        {
            int id = request->motor_id[i];
            response->present_pos[i] = motor_drivers_[id]->present_data.position;
            response->present_vel[i] = motor_drivers_[id]->present_data.velocity;
            response->present_tor[i] = motor_drivers_[id]->present_data.torque;
        }
    }

    void timer_callback()
    {
        MotorDriver::get_frame();
        for (auto& driver: motor_drivers_) driver->process_rx();
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);

        std::vector<int64_t> motor_ids;
        std::vector<std::string> motor_names;
        std::vector<int64_t> motor_brands;
        std::vector<int64_t> motor_types;
        motor_ids = this->declare_parameter("motor_ids", motor_ids);
        motor_names = this->declare_parameter("motor_names", motor_names);
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        motor_types = this->declare_parameter("motor_types", motor_types);

        // create corresponding drivers
        for (int i = 0; i < motor_count; i++)
        {
            int id = motor_ids[i];
            if (motor_brands[i] == DaMiao)
            {
                motor_drivers_[i] = std::make_unique<DmMotorDriver>(id);
            }
            else if (motor_brands[i] == DJI)
            {
                motor_drivers_[i] = std::make_unique<DjiMotorDriver>(id, (MotorType)motor_types[i]);
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