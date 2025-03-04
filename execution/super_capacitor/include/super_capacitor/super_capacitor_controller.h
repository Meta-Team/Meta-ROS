#ifndef SUPER_CAPACITOR_CONTROLLER_H
#define SUPER_CAPACITOR_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <chrono>
#include <unordered_map>
#include <atomic>

#include "super_capacitor/super_capacitor_base.h"
#include "device_interface/msg/capacitor_cmd.hpp"
#include "device_interface/msg/capacitor_state.hpp"
#include "meta_hardware/can_driver/can_driver.hpp"

class SuperCapacitorController
{
public:
    SuperCapacitorController(const rclcpp::NodeOptions & options);
    ~SuperCapacitorController();
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<super_capacitor::SuperCapacitorBase> capacitor_; 
    // double target_power_, referee_power_;
    std::atomic<double> target_power_;
    std::atomic<double> referee_power_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr tx_timer_;

    rclcpp::Publisher<device_interface::msg::CapacitorState>::SharedPtr state_pub_;
    rclcpp::Subscription<device_interface::msg::CapacitorCmd>::SharedPtr goal_sub_;

    void goal_sub_callback(const device_interface::msg::CapacitorCmd::SharedPtr msg);
    void send_command();
    void pub_state();
};

#endif  // SUPER_CAPACITOR_CONTROLLER_H