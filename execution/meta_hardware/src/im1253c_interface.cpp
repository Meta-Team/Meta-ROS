#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "meta_hardware/im1253c_interface.hpp"
#include "meta_hardware/motor_network/mi_motor_network.hpp"
#include "rclcpp/rclcpp.hpp"

namespace meta_hardware {
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

MetaRobotIm1253cManager::~MetaRobotIm1253cManager() = default;

hardware_interface::CallbackReturn
MetaRobotIm1253cManager::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    // joint_interface_data_.resize(info_.joints.size());
    // joint_motor_info_.resize(info_.joints.size());

    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MetaRobotIm1253cManager::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    std::vector<std::unordered_map<std::string, std::string>> joint_params;
    // Add the motors to the motor networks
    // for (size_t i = 0; i < info_.joints.size(); ++i) {
    //     const auto &joint = info_.joints[i];
    //     const auto &joint_param = joint.parameters;
    //     joint_motor_info_[i].name = joint.name;
    //     joint_motor_info_[i].mechanical_reduction =
    //         std::stod(joint_param.at("mechanical_reduction"));
    //     joint_motor_info_[i].offset = std::stod(joint_param.at("offset"));
    //     joint_motor_info_[i].mode = check_motor_mode(
    //         joint_param.at("control_mode"), joint_motor_info_[i].command_pos,
    //         joint_motor_info_[i].command_vel, joint_motor_info_[i].command_eff);
    //     joint_params.emplace_back(joint_param);
    // }

    // std::string can_network_name = info_.hardware_parameters.at("can_network_name");
    // mi_motor_network_ =
    //     std::make_unique<MiMotorNetwork>(can_network_name, 0x00, joint_params);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetaRobotIm1253cManager::export_state_interfaces() {
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

    // for (size_t i = 0; i < info_.joints.size(); ++i) {
    //     const auto &joint_state_interfaces = info_.joints[i].state_interfaces;
    //     if (contains_interface(joint_state_interfaces, "position")) {
    //         state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
    //                                       &joint_interface_data_[i].state_position);
    //     }
    //     if (contains_interface(joint_state_interfaces, "velocity")) {
    //         state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
    //                                       &joint_interface_data_[i].state_velocity);
    //     }
    //     if (contains_interface(joint_state_interfaces, "effort")) {
    //         state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
    //                                       &joint_interface_data_[i].state_effort);
    //     }
    // }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetaRobotIm1253cManager::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Helper function to check if the interface exists
    // auto contains_interface =
    //     [](const std::vector<hardware_interface::InterfaceInfo> &interfaces,
    //        const std::string &interface_name) {
    //         return std::ranges::find_if(
    //                    interfaces,
    //                    [&interface_name](
    //                        const hardware_interface::InterfaceInfo &interface) {
    //                        return interface.name == interface_name;
    //                    }) != interfaces.end();
    //     };

    // for (size_t i = 0; i < info_.joints.size(); ++i) {
    //     const auto &joint_command_interfaces = info_.joints[i].command_interfaces;
    //     if (contains_interface(joint_command_interfaces, "position")) {
    //         command_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
    //                                         &joint_interface_data_[i].command_position);
    //         joint_motor_info_[i].command_pos = true;
    //     }
    //     if (contains_interface(joint_command_interfaces, "velocity")) {
    //         command_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
    //                                         &joint_interface_data_[i].command_velocity);
    //         joint_motor_info_[i].command_vel = true;
    //     }
    //     if (contains_interface(joint_command_interfaces, "effort")) {
    //         command_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
    //                                         &joint_interface_data_[i].command_effort);
    //         joint_motor_info_[i].command_eff = true;
    //     }
    // }

    return command_interfaces;
}

hardware_interface::CallbackReturn
MetaRobotIm1253cManager::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetaRobotIm1253cManager::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MetaRobotIm1253cManager::read(const rclcpp::Time & /*time*/,
                              const rclcpp::Duration & /*period*/) {

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MetaRobotIm1253cManager::write(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {
    return hardware_interface::return_type::OK;
}

} // namespace meta_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_hardware::MetaRobotIm1253cManager,
                       hardware_interface::SystemInterface)
