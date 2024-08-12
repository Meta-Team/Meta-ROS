#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "meta_hardware/brt_encoder_interface.hpp"
#include "meta_hardware/motor_network/brt_encoder_network.hpp"

namespace meta_hardware {
using hardware_interface::HW_IF_POSITION;

MetaRobotBrtEncoderNetwork::~MetaRobotBrtEncoderNetwork() = default;

hardware_interface::CallbackReturn
MetaRobotBrtEncoderNetwork::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    joint_interface_data_.resize(info_.joints.size());
    joint_motor_info_.resize(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetaRobotBrtEncoderNetwork::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    std::vector<std::unordered_map<std::string, std::string>> joint_params;
    // Add the motors to the motor networks
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        const auto &joint_param = joint.parameters;
        joint_motor_info_[i].name = joint.name;
        joint_motor_info_[i].mechanical_reduction =
            std::stod(joint_param.at("mechanical_reduction"));
        joint_motor_info_[i].offset = std::stod(joint_param.at("offset"));
        joint_params.emplace_back(joint_param);
    }

    std::string can_network_name = info_.hardware_parameters.at("can_network_name");
    brt_encoder_network_ =
        std::make_unique<BrtEncoderNetwork>(can_network_name, joint_params);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetaRobotBrtEncoderNetwork::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Helper function to check if the interface exists
    auto contains_interface =
        [](const std::vector<hardware_interface::InterfaceInfo> &interfaces,
           const std::string &interface_name) {
            return std::ranges::find_if(
                       interfaces,
                       [&interface_name](
                           const hardware_interface::InterfaceInfo &interface) {
                           return interface.name == interface_name;
                       }) != interfaces.end();
        };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_state_interfaces = info_.joints[i].state_interfaces;
        if (contains_interface(joint_state_interfaces, "position")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
                                          &joint_interface_data_[i].state_position);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetaRobotBrtEncoderNetwork::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // No command interfaces for encoder

    return command_interfaces;
}

hardware_interface::CallbackReturn MetaRobotBrtEncoderNetwork::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetaRobotBrtEncoderNetwork::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MetaRobotBrtEncoderNetwork::read(const rclcpp::Time & /*time*/,
                                 const rclcpp::Duration & /*period*/) {

    for (size_t i = 0; i < joint_motor_info_.size(); ++i) {
        auto position = brt_encoder_network_->read(i);

        double reduction = joint_motor_info_[i].mechanical_reduction;
        double offset = joint_motor_info_[i].offset;
        position = position / reduction + offset;

        joint_interface_data_[i].state_position = position;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MetaRobotBrtEncoderNetwork::write(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {
    // No command interfaces for encoder

    return hardware_interface::return_type::OK;
}

} // namespace meta_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_hardware::MetaRobotBrtEncoderNetwork,
                       hardware_interface::SystemInterface)
