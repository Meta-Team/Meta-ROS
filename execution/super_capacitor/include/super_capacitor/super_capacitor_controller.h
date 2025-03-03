#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <chrono>
#include <unordered_map>

#include "super_capacitor_base.h"
#include "device_interface/msg/super_capacitor_goal.hpp"
#include "device_interface/msg/super_capacitor_state.hpp"
#include "can_driver/can_driver.hpp"


class SuperCapacitorController : public rclcpp::Node
{
public:
    SuperCapacitorController();

private:
    std::shared_ptr<SuperCapacitorBase> capacitor_; 
    double target_power_, referee_power_;
    void goal_sub_callback(const device_interface::msg::SuperCapacitorGoal::SharedPtr msg);
    void SuperCapacitorController::send_command();
    void SuperCapacitorController:pub_state();
};