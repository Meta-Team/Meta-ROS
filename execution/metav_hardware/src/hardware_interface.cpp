#include <iostream>
#include <limits>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "metav_hardware/hardware_interface.hpp"
#include "metav_hardware/motor_network/dji_motor_network.hpp"
#include "rclcpp/rclcpp.hpp"

namespace metav_hardware {
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

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

    for (const auto &joint : info_.joints) {
        std::string motor_vendor = joint.parameters.at("motor_vendor");
        std::string can_network_name = joint.parameters.at("can_network_name");
        // Allocate the motor network if it doesn't exist
        if (can_motor_networks_[motor_vendor].find(can_network_name) ==
            can_motor_networks_[motor_vendor].end()) {
            if (motor_vendor == "DJI") {
                can_motor_networks_[motor_vendor][can_network_name] =
                    std::make_shared<DjiMotorNetwork>(can_network_name);
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("MetavRobotHardwareInterface"),
                             "Unknown motor vendor: %s", motor_vendor.c_str());
                return CallbackReturn::ERROR;
            }
        }
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint = info_.joints[i];
        std::string motor_vendor = joint.parameters.at("motor_vendor");
        std::string motor_model = joint.parameters.at("motor_model");
        std::string can_network_name = joint.parameters.at("can_network_name");
        auto motor_id =
            static_cast<uint32_t>(std::stoul(joint.parameters.at("motor_id")));
        can_motor_networks_[motor_vendor][can_network_name]->add_motor(
            motor_model, motor_id, i);
        joint_motors_[i] =
            JointMotor{.name = joint.name,
                       .motor_vendor = motor_vendor,
                       .motor_model = motor_model,
                       .can_network_name = can_network_name,
                       .motor_id = motor_id,
                       .can_motor_network =
                           can_motor_networks_[motor_vendor][can_network_name]};
    }

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetavRobotHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_state_interfaces = info_.joints[i].state_interfaces;
        if (std::find_if(
                joint_state_interfaces.begin(), joint_state_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "position";
                }) != joint_state_interfaces.end()) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
                                          &hw_states_[i * 3 + 0]);
        }
        if (std::find_if(
                joint_state_interfaces.begin(), joint_state_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "velocity";
                }) != joint_state_interfaces.end()) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                          &hw_states_[i * 3 + 1]);
        }
        if (std::find_if(
                joint_state_interfaces.begin(), joint_state_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "effort";
                }) != joint_state_interfaces.end()) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                          &hw_states_[i * 3 + 2]);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetavRobotHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_command_interfaces =
            info_.joints[i].command_interfaces;
        if (std::find_if(
                joint_command_interfaces.begin(),
                joint_command_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "position";
                }) != joint_command_interfaces.end()) {
            command_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_POSITION, &hw_commands_[i * 3 + 0]);
        }
        if (std::find_if(
                joint_command_interfaces.begin(),
                joint_command_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "velocity";
                }) != joint_command_interfaces.end()) {
            command_interfaces.emplace_back(
                info_.joints[i].name, HW_IF_VELOCITY, &hw_commands_[i * 3 + 1]);
        }
        if (std::find_if(
                joint_command_interfaces.begin(),
                joint_command_interfaces.end(),
                [](const hardware_interface::InterfaceInfo &interface) {
                    return interface.name == "effort";
                }) != joint_command_interfaces.end()) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_EFFORT,
                                            &hw_commands_[i * 3 + 2]);
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {

    for (const auto &can_motor_network : can_motor_networks_) {
        for (const auto &motor_network : can_motor_network.second) {
            motor_network.second->init_rx();
        }
    }

    return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MetavRobotHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    // TODO(anyone): prepare the robot to stop receiving commands

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
        double effort = hw_commands_[i * 3 + 2];
        if (std::isnan(effort)) {
            continue;
        }
        joint_motors_[i].can_motor_network->write(i, effort);
    }

    for (const auto &can_motor_network : can_motor_networks_) {
        for (const auto &motor_network : can_motor_network.second) {
            motor_network.second->tx();
        }
    }
    return hardware_interface::return_type::OK;
}

} // namespace metav_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(metav_hardware::MetavRobotHardwareInterface,
                       hardware_interface::SystemInterface)
