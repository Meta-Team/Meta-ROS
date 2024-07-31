#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "metav_hardware/hardware_interface.hpp"
#include "metav_hardware/motor_network/dji_motor_network.hpp"
#include "metav_hardware/motor_network/mi_motor_network.hpp"
#include "rclcpp/rclcpp.hpp"

namespace metav_hardware {
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

MetavRobotHardwareInterface::~MetavRobotHardwareInterface() {
    on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }

    hw_states_.resize(info_.joints.size() * 3,
                      std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size() * 3,
                        std::numeric_limits<double>::quiet_NaN());
    joint_motors_.resize(info_.joints.size());

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        joint_motors_[i].name = info_.joints[i].name;
        joint_motors_[i].motor_vendor =
            info_.joints[i].parameters.at("motor_vendor");
        joint_motors_[i].can_network_name =
            info_.joints[i].parameters.at("can_network_name");
    }

    // Initialize the motor networks with joint information
    for (const auto &joint : info_.joints) {
        std::string motor_vendor = joint.parameters.at("motor_vendor");
        std::string can_network_name = joint.parameters.at("can_network_name");
        // Allocate the motor network if it doesn't exist
        if (can_motor_networks_[motor_vendor].find(can_network_name) ==
            can_motor_networks_[motor_vendor].end()) {
            if (motor_vendor == "DJI") {
                can_motor_networks_[motor_vendor][can_network_name] =
                    std::make_shared<DjiMotorNetwork>(can_network_name);
            } else if (motor_vendor == "MI") {
                can_motor_networks_[motor_vendor][can_network_name] =
                    std::make_shared<MiMotorNetwork>(can_network_name, 0x00);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("MetavRobotHardwareInterface"),
                             "Unknown motor vendor: %s", motor_vendor.c_str());
                return CallbackReturn::ERROR;
            }
        }
    }

    // Add the motors to the motor networks
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        const auto &joint_params = joint.parameters;
        std::string motor_vendor = joint_params.at("motor_vendor");
        std::string can_network_name = joint_params.at("can_network_name");

        can_motor_networks_[joint_motors_[i].motor_vendor]
                           [joint_motors_[i].can_network_name]
                               ->add_motor(i, joint_params);

        joint_motors_[i].can_motor_network =
            can_motor_networks_[motor_vendor][can_network_name];
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetavRobotHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Helper function to check if the interface exists
    auto contains_interface =
        [](const std::vector<hardware_interface::InterfaceInfo> &interfaces,
           const std::string &interface_name) {
            return std::find_if(
                       interfaces.begin(), interfaces.end(),
                       [&interface_name](
                           const hardware_interface::InterfaceInfo &interface) {
                           return interface.name == interface_name;
                       }) != interfaces.end();
        };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_state_interfaces = info_.joints[i].state_interfaces;
        if (contains_interface(joint_state_interfaces, "position")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
                                          &hw_states_[i * 3 + 0]);
        }
        if (contains_interface(joint_state_interfaces, "velocity")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                          &hw_states_[i * 3 + 1]);
        }
        if (contains_interface(joint_state_interfaces, "effort")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                          &hw_states_[i * 3 + 2]);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetavRobotHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Helper function to check if the interface exists
    auto contains_interface =
        [](const std::vector<hardware_interface::InterfaceInfo> &interfaces,
           const std::string &interface_name) {
            return std::find_if(
                       interfaces.begin(), interfaces.end(),
                       [&interface_name](
                           const hardware_interface::InterfaceInfo &interface) {
                           return interface.name == interface_name;
                       }) != interfaces.end();
        };

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_command_interfaces =
            info_.joints[i].command_interfaces;
        if (contains_interface(joint_command_interfaces, "position")) {
            command_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_POSITION, &hw_commands_[i * 3 + 0]);
            joint_motors_[i].command_pos = true;
        }
        if (contains_interface(joint_command_interfaces, "velocity")) {
            command_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i * 3 + 1]);
            joint_motors_[i].command_vel = true;
        }
        if (contains_interface(joint_command_interfaces, "effort")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                            &hw_commands_[i * 3 + 2]);
            joint_motors_[i].command_eff = true;
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    // Clear all the shared pointers to fully deallocate the motor networks
    // This will trigger the destructor of the motor networks
    for (auto &[motor_vendor, can_motor_networks] : can_motor_networks_) {
        can_motor_networks.clear();
    }

    for (auto &joint_motor : joint_motors_) {
        joint_motor.can_motor_network = nullptr;
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MetavRobotHardwareInterface::read(const rclcpp::Time & /*time*/,
                                  const rclcpp::Duration & /*period*/) {

    for (size_t i = 0; i < joint_motors_.size(); ++i) {
        auto [position, velocity, effort] =
            joint_motors_[i].can_motor_network->read(i);
        hw_states_[i * 3 + 0] = position;
        hw_states_[i * 3 + 1] = velocity;
        hw_states_[i * 3 + 2] = effort;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MetavRobotHardwareInterface::write(const rclcpp::Time & /*time*/,
                                   const rclcpp::Duration & /*period*/) {

    for (size_t i = 0; i < joint_motors_.size(); ++i) {
        double position = hw_commands_[i * 3 + 0];
        double velocity = hw_commands_[i * 3 + 1];
        double effort = hw_commands_[i * 3 + 2];

        // Check if the command is valid
        // If a command interface exists, the command must not be NaN
        if (joint_motors_[i].command_pos && std::isnan(position)) {
            continue;
        }
        if (joint_motors_[i].command_vel && std::isnan(velocity)) {
            continue;
        }
        if (joint_motors_[i].command_eff && std::isnan(effort)) {
            continue;
        }

        // Write the command to the motor network
        joint_motors_[i].can_motor_network->write(i, position, velocity,
                                                  effort);
    }

    // Some motor network implementations require a separate tx() call
    for (const auto &[motor_vendor, can_motor_networks] : can_motor_networks_) {
        for (const auto &[can_network_name, can_motor_network] :
             can_motor_networks) {
            can_motor_network->tx();
        }
    }

    return hardware_interface::return_type::OK;
}

} // namespace metav_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(metav_hardware::MetavRobotHardwareInterface,
                       hardware_interface::SystemInterface)
