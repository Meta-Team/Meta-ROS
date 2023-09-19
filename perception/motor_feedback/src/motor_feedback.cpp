#include "motor_feedback/motor_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <bits/stdint-intn.h>
#include <vector>

#include "motor_feedback/motor_driver.hpp"
#include "motor_feedback/dm_motor_driver.h"
#include "motor_feedback/dji_motor_driver.h"

#include "motor_interface/srv/motor_present.hpp"

#define DaMiao 0
#define DJI 1

can_frame MotorDriver::rx_frame;
CanDriver* MotorDriver::can_0 = new CanDriver(0);

class MotorFeedback : public rclcpp::Node
{
private:
    MotorData present_data[16]; // not fully used, up to 16 motors

    rclcpp::Service<motor_interface::srv::MotorPresent>::SharedPtr srv_;

    void srv_callback(const motor_interface::srv::MotorPresent::Request::SharedPtr request,
                      motor_interface::srv::MotorPresent::Response::SharedPtr response)
    {
        int feedback_count = request->motor_id.size();
        for (int i = 0; i < feedback_count; i++)
        {
            // int id = request->motor_id[i];
            response->present_pos[i] = present_data[request->motor_id[i]].position;
            response->present_vel[i] = present_data[request->motor_id[i]].velocity;
            response->present_tor[i] = present_data[request->motor_id[i]].torque;
        }
    }

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
    }

    ~MotorFeedback()
    {
        for (int i = 0; i < motor_count; i++)
        {
            delete motor_drivers_[i];
        }
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

    void update_data()
    {
        int rx_id; // received id
        MotorDriver::update_rx(rx_id);

        present_data[rx_id] = motor_drivers_[rx_id]->process_rx();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto motor_feedback_ = std::make_shared<MotorFeedback>();

    // Start a new thread to run the update_data function
    std::thread update_thread([&motor_feedback_](){
        while(rclcpp::ok())
        {
            motor_feedback_->update_data();
            rclcpp::sleep_for(std::chrono::milliseconds(1));
        }
    });

    // Run the ROS 2 event loop in the main thread
    rclcpp::spin(motor_feedback_);

    // Wait for the update thread to finish
    update_thread.join();

    rclcpp::shutdown();
    return 0;
}