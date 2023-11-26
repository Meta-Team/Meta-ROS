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

can_frame MotorDriver::rx_frame;
std::shared_ptr<CanDriver> MotorDriver::can_0 = std::make_shared<CanDriver>(0);

// TODO: id and i remain to be distinguished

class MotorFeedback : public rclcpp::Node
{
private:
    MotorData present_data[16]; // not fully used, up to 16 motors
    // in the order of motor_id, based on config file

    rclcpp::Service<motor_interface::srv::MotorPresent>::SharedPtr srv_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    int motor_count;
    MotorDriver* motor_drivers_[16]; // not fully used, up to 16 motors

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

    ~MotorFeedback()
    {
        for (int i = 0; i < motor_count; i++)
        {
            delete motor_drivers_[i];
        }
    }

    void srv_callback(const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                      motor_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        int feedback_count = request->motor_id.size();
        for (int i = 0; i < feedback_count; i++)
        {
            int id = request->motor_id[i];
            response->present_pos[i] = present_data[id].position;
            response->present_vel[i] = present_data[id].velocity;
            response->present_tor[i] = present_data[id].torque;
        }
    }

    void timer_callback()
    {
        MotorDriver::can_0->get_frame(MotorDriver::rx_frame);
        int rx_id = can2index(MotorDriver::rx_frame.can_id);

        present_data[rx_id] = motor_drivers_[rx_id]->process_rx();
    }

    void motor_init()
    {
        motor_count = this->declare_parameter("motor_count", motor_count);

        std::vector<std::string> motor_names;
        std::vector<int64_t> motor_brands;
        std::vector<int64_t> motor_types;
        motor_names = this->declare_parameter("motor_names", motor_names);
        motor_brands = this->declare_parameter("motor_brands", motor_brands);
        motor_types = this->declare_parameter("motor_types", motor_types);

        // create corresponding drivers
        for (int i = 0; i < motor_count; i++)
        {
            if (motor_brands[i] == DaMiao)
            {
                motor_drivers_[i] = new DmMotorDriver();
            }
            else if (motor_brands[i] == DJI)
            {
                motor_drivers_[i] = new DjiMotorDriver();
            }
        }
    }

    int can2index(int can_id)
    {
        // TODO: to be further designed
        return can_id - 0x200;
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