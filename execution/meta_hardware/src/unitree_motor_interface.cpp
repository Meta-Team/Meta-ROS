#include <iostream>
#include <limits>
#include <memory>
#include <vector>
#include <ranges>
#include <cmath>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "meta_hardware/unitree_motor_interface.hpp"
#include "meta_hardware/motor_network/unitree_motor_network.hpp"

namespace meta_hardware {
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

MetaRobotUnitreeMotorInterface::~MetaRobotUnitreeMotorInterface() = default;

hardware_interface::CallbackReturn
MetaRobotUnitreeMotorInterface::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    joint_interface_data_.resize(info_.joints.size());
    joint_motors_info_.resize(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetaRobotUnitreeMotorInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    std::vector<std::unordered_map<std::string, std::string>> joint_params;

    // Add the motors to the motor networks
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        const auto &joint_param = joint.parameters;
        
        joint_motors_info_[i].joint_name = joint.name;
        joint_motors_info_[i].mechanical_reduction = std::stod(joint_param.at("mechanical_reduction"));
        joint_motors_info_[i].offset = std::stod(joint_param.at("offset"));
        
        joint_params.emplace_back(joint_param);
    }

    std::string tty_devpath = info_.hardware_parameters.at("tty_devpath");
    
    // Initialize the network with the serial port path and all joint parameters
    unitree_motor_network_ = std::make_unique<UnitreeMotorNetwork>(tty_devpath, joint_params);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetaRobotUnitreeMotorInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

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
        if (contains_interface(joint_state_interfaces, "velocity")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                          &joint_interface_data_[i].state_velocity);
        }
        if (contains_interface(joint_state_interfaces, "effort")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                          &joint_interface_data_[i].state_effort);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetaRobotUnitreeMotorInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

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
        const auto &joint_command_interfaces = info_.joints[i].command_interfaces;
        
        // Always export all MIT interfaces
        if (contains_interface(joint_command_interfaces, "position")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
                                            &joint_interface_data_[i].command_position);
        }
        if (contains_interface(joint_command_interfaces, "velocity")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                            &joint_interface_data_[i].command_velocity);
        }
        if (contains_interface(joint_command_interfaces, "effort")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                            &joint_interface_data_[i].command_effort);
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn MetaRobotUnitreeMotorInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetaRobotUnitreeMotorInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MetaRobotUnitreeMotorInterface::read(const rclcpp::Time & /*time*/,
                               const rclcpp::Duration & /*period*/) {

    // Sync: Send previous commands, receive current states (Blocking I/O)
    unitree_motor_network_->sync_read_write();

    for (size_t i = 0; i < joint_motors_info_.size(); ++i) {
        auto [position, velocity, effort] = unitree_motor_network_->read_state(i);

        double reduction = joint_motors_info_[i].mechanical_reduction;
        double offset = joint_motors_info_[i].offset;

        // Apply reduction and offset
        position = position / reduction + offset;
        velocity /= reduction;
        effort *= reduction;
        printf("q:%.2lf\n", position);
        joint_interface_data_[i].state_position = position;
        joint_interface_data_[i].state_velocity = velocity;
        joint_interface_data_[i].state_effort = effort;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MetaRobotUnitreeMotorInterface::write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {

    for (size_t i = 0; i < joint_motors_info_.size(); ++i) {
        double position = joint_interface_data_[i].command_position;
        double velocity = joint_interface_data_[i].command_velocity;
        double effort = joint_interface_data_[i].command_effort;

        double reduction = joint_motors_info_[i].mechanical_reduction;
        double offset = joint_motors_info_[i].offset;

        // Convert to motor space
        position = (position - offset) * reduction;
        velocity *= reduction;
        effort /= reduction;

        // Note that NaN checks are within motor_driver, if NaN then set motor mode to 0(BREAK)

        // Update the command buffer in the network (will be sent in next read() call)
        unitree_motor_network_->write_command(i, position, velocity, effort);
    }

    return hardware_interface::return_type::OK;
}

} // namespace meta_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_hardware::MetaRobotUnitreeMotorInterface,
                       hardware_interface::SystemInterface)