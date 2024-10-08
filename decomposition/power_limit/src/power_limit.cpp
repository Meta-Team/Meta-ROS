#include "power_limit/power_limit.hpp"

#include <controller_interface/controller_interface.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace power_limit {

PowerLimitController::PowerLimitController()
    : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn PowerLimitController::on_init() {

    try {
        param_listener_ =
            std::make_shared<power_limit::ParamListener>(get_node());
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown during controller's init with message: %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PowerLimitController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();
   
    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<MotorPowerMsg>(
            "~/state", rclcpp::SystemDefaultsQoS());
        state_publisher_ = std::make_unique<MotorPowerPublisher>(s_publisher_);
    } catch (const std::exception &e) {
        fprintf(stderr,
                "Exception thrown during publisher creation at configure stage "
                "with message : %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
PowerLimitController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
PowerLimitController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(3);
    state_interfaces_config.names.push_back(params_.name + "/voltage");
    state_interfaces_config.names.push_back(params_.name + "/current");
    state_interfaces_config.names.push_back(params_.name + "/power");
    // state_interfaces_config.names.push_back(params_.motor_joint + "/" + hardware_interface::HW_IF_VELOCITY);
    // state_interfaces_config.names.push_back(params_.motor_joint + "/" + hardware_interface::HW_IF_EFFORT);
    return state_interfaces_config;
}

controller_interface::CallbackReturn
PowerLimitController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PowerLimitController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
PowerLimitController::update(const rclcpp::Time &/*time*/,
                                                 const rclcpp::Duration &/*period*/) {
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }

    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.voltage = state_interfaces_[0].get_value();
        state_publisher_->msg_.current = state_interfaces_[1].get_value();
        state_publisher_->msg_.power = state_interfaces_[2].get_value();
        // state_publisher_->msg_.motor_velocity = state_interfaces_[3].get_value();
        // state_publisher_->msg_.motor_effort = state_interfaces_[4].get_value();
        state_publisher_->msg_.motor_velocity = 0.0;
        state_publisher_->msg_.motor_effort = 0.0;
        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}



} // namespace meta_chassis_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(power_limit::PowerLimitController,
                       controller_interface::ControllerInterface)
