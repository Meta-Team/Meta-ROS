#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>

#include "device_interface/msg/super_capacitor_goal.hpp"
#include "device_interface/msg/super_capacitor_state.hpp"
#include "can_driver/can_driver.hpp"

class SuperCapacitorController : public rclcpp::Node
{
public:
    SuperCapacitorController() : Node("SuperCapacitorController")
    {

        // get parameters
        std::string can_interface = this->declare_parameter("can_interface", std::string("can0"));
        std::vector<int> can_filters;

        // initialize the super capacitors
        can_driver_ = std::make_unique<CanDriver>(can_interface, false, can_filters);

        // create subscriptions
        goal_sub_ = this->create_subscription<device_interface::msg::SuperCapacitorGoal>(
            "super_capacitor_goal", 10, [this](const device_interface::msg::SuperCapacitorGoal::SharedPtr msg){
                goal_sub_callback(msg);
            });

        // create a timer for publishing super capacitor state
        state_pub_ = this->create_publisher<device_interface::msg::SuperCapacitorState>("super_capacitor_state", 10);
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUB_R), [this](){
                pub_timer_callback();
            });

        // complete initialization
        tx_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), [this](){
                tx();
            });
        RCLCPP_INFO(this->get_logger(), "SuperCapacitorController initialized");
    }

private:
    void SuperCpacitorController::goal_sub_callback(const device_interface::msg::SuperCapacitorGoal::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(), "Received super capacitor goal");
        msg->
    }
    void SuperCapacitorController::tx() {
        try {
            for (auto &frame_id : {0x1fe, 0x1ff, 0x200, 0x2fe, 0x2ff}) {
                if (tx_frames_.contains(frame_id)) {
                    can_driver_->write(tx_frames_[frame_id]);
                }
            }
        } catch (CanIOException &e) {
            std::cerr << "Error writing CAN message: " << e.what() << std::endl;
        }
    }
    
};