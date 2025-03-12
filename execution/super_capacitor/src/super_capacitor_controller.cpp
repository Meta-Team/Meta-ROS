#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <memory>
#include <string>
#include <pluginlib/class_loader.hpp>
#include <chrono>
#include <unordered_map>

#include "super_capacitor/super_capacitor_base.h"
#include "super_capacitor/super_capacitor_controller.h"
#include "meta_hardware/can_driver/can_driver.hpp"
#include "super_capacitor/xidi_capacitor_driver.h"
#include "super_capacitor/pacific_spirit_capacitor_driver.h"


SuperCapacitorController::SuperCapacitorController(const rclcpp::NodeOptions & options)
{
    node_ = rclcpp::Node::make_shared("super_capacitor_controller", options);

    // get parameters
    std::string can_interface = node_->declare_parameter("can_interface", std::string("can0"));
    std::string capacitor_device = node_->declare_parameter("capacitor_device", std::string("xidi"));
    // create subscriptions
    // goal_sub_ = node_->create_subscription<device_interface::msg::CapacitorCmd>(
    //     "super_capacitor_goal", 10, [node_](const device_interface::msg::CapacitorCmd::SharedPtr msg){
    //         goal_sub_callback(msg);
    //     });
    goal_sub_ = node_->create_subscription<device_interface::msg::CapacitorCmd>(
        "super_capacitor_goal", 10, std::bind(&SuperCapacitorController::goal_sub_callback, this, std::placeholders::_1));
    // create a timer for publishing super capacitor state
    state_pub_ = node_->create_publisher<device_interface::msg::CapacitorState>("super_capacitor_state", 10);
    pub_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SuperCapacitorController::pub_state, this));

    // complete initialization
    tx_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&SuperCapacitorController::send_command, this));


    // pluginlib::ClassLoader<super_capacitor::SuperCapacitorBase> capacitor_loader("super_capacitor","super_capacitor::SuperCapacitorBase");

    try
    {
        // capacitor_ = capacitor_loader.createSharedInstance("super_capacitor::XidiCapacitorDriver");
        // capacitor_->init(can_interface);
        if(capacitor_device == "xidi"){
            capacitor_ = std::make_shared<super_capacitor::XidiCapacitorDriver>();
            capacitor_->init(can_interface);
        }else if(capacitor_device == "pacific_spirit"){
            capacitor_ = std::make_shared<super_capacitor::PacificSpiritCapacitorDriver>();
            capacitor_->init(can_interface);
        }else{
            RCLCPP_ERROR(node_->get_logger(), "Unknown capacitor device: %s", capacitor_device.c_str());
        }
        
    }
    catch(pluginlib::PluginlibException& ex)
    {
        printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
    }
    RCLCPP_INFO(node_->get_logger(), "SuperCapacitorController initialized");
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SuperCapacitorController::get_node_base_interface() const
{
    return node_->get_node_base_interface();
}

void SuperCapacitorController::goal_sub_callback(const device_interface::msg::CapacitorCmd::SharedPtr msg){
    target_power_.store(msg->target_power,std::memory_order_relaxed);
    referee_power_.store(msg->referee_power,std::memory_order_relaxed);
    capacitor_->set_target_power(target_power_.load(std::memory_order_relaxed));
    capacitor_->set_referee_power(referee_power_.load(std::memory_order_relaxed));
}

void SuperCapacitorController::send_command() {
    capacitor_->tx();
}

void SuperCapacitorController::pub_state() {
    std::unordered_map<std::string, double> state = capacitor_->get_state();
    device_interface::msg::CapacitorState msg;
    if (state.contains("input_voltage"))  msg.input_voltage = state.at("input_voltage");  
    if (state.contains("capacitor_voltage")) msg.capacitor_voltage = state.at("capacitor_voltage");   
    if (state.contains("input_current")) msg.input_current = state.at("input_current");
    if (state.contains("target_power")) msg.target_power = state.at("target_power");
    if (state.contains("max_discharge_power")) msg.max_discharge_power = state.at("max_discharge_power");
    if (state.contains("base_power")) msg.base_power = state.at("base_power");
    if (state.contains("cap_energy_percentage")) msg.cap_energy_percentage = state.at("cap_energy_percentage");
    if (state.contains("cap_state")) msg.cap_state = state.at("cap_state");
    state_pub_->publish(msg);
}

SuperCapacitorController::~SuperCapacitorController() {
    // Destructor implementation
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SuperCapacitorController)