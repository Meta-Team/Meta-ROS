#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "meta_hardware/unitree_motor_interface.hpp"
// #include "meta_hardware/motor_network/dji_motor_network.hpp"

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

    std::vector<std::unordered_map<std::string, std::string>> motor_params;

    // Add the motors to the motor networks
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const auto &joint_params = info_.joints[i].parameters;
        motor_params.push_back(joint_params);
        joint_motors_info_[i].joint_name = info_.joints[i].name;
        // joint_motors_info_[i].mechanical_reduction =
        //     std::stod(joint_params.at("mechanical_reduction"));
        // joint_motors_info_[i].offset = std::stod(joint_params.at("offset"));
    }
    
    // TODO
    std::string tty_devpath = info_.hardware_parameters.at("tty_devpath");
    tmp_unitree_serialport = std::make_unique<SerialPort>(tty_devpath);
    tmp_unitree_cmd = std::make_unique<MotorCmd>();
    tmp_unitree_data = std::make_unique<MotorData>();

    tmp_unitree_cmd->motorType = MotorType::GO_M8010_6;
    tmp_unitree_data->motorType = MotorType::GO_M8010_6;
    tmp_unitree_cmd->mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
    // should be set in configure but not here
    tmp_unitree_cmd->id = 0;
    // position control
    // tmp_unitree_cmd->kp = 0.01;
    // tmp_unitree_cmd->kd = 0.01;
    // tmp_unitree_cmd->q = 0.0;
    // tmp_unitree_cmd->dq = 0.0;
    // tmp_unitree_cmd->tau = 0.0;
    // velocity control
    tmp_unitree_cmd->kp = 0.0;
    tmp_unitree_cmd->kd = 0.01;
    tmp_unitree_cmd->q = 0.0;
    tmp_unitree_cmd->dq = 0.0;
    tmp_unitree_cmd->tau = 0.0;
    // tmp_unitree_cmd
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
MetaRobotUnitreeMotorInterface::export_state_interfaces() {
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
        if (contains_interface(joint_state_interfaces, "velocity")) {
            state_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                          &joint_interface_data_[i].state_velocity);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetaRobotUnitreeMotorInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

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
        const auto &joint_command_interfaces = info_.joints[i].command_interfaces;
        if (contains_interface(joint_command_interfaces, "position")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_POSITION,
                                            &joint_interface_data_[i].command_position);
        }
        if (contains_interface(joint_command_interfaces, "velocity")) {
            command_interfaces.emplace_back(info_.joints[i].name, HW_IF_VELOCITY,
                                            &joint_interface_data_[i].command_velocity);
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

    // for (size_t i = 0; i < joint_motors_info_.size(); ++i) {
    //     auto [position, velocity, effort] = dji_motor_network_->read(i);

    //     double reduction = joint_motors_info_[i].mechanical_reduction;
    //     double offset = joint_motors_info_[i].offset;
    //     position = position / reduction + offset;
    //     velocity /= reduction;
    //     if (reduction < 0) {
    //         effort = -effort;
    //     }
        
        joint_interface_data_[0].state_position = tmp_unitree_data->q;
        joint_interface_data_[0].state_velocity = tmp_unitree_data->dq;
        // joint_interface_data_[i].state_effort = effort;
    // }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MetaRobotUnitreeMotorInterface::write(const rclcpp::Time & /*time*/,
                                const rclcpp::Duration & /*period*/) {

    // for (size_t i = 0; i < joint_motors_info_.size(); ++i) {
    // for (size_t i = 0; i < 1; ++i) {
        double position = joint_interface_data_[0].command_position;
        double velocity = joint_interface_data_[0].command_velocity;

        // Check if the command is valid
        // If a command interface exists, the command must not be NaN
        // position control
        // if (!std::isnan(position)) {
        //     tmp_unitree_cmd->q = position;
        // }
        if (!std::isnan(velocity)) {
            tmp_unitree_cmd->dq = velocity*queryGearRatio(MotorType::GO_M8010_6);
        }
        tmp_unitree_serialport->sendRecv(tmp_unitree_cmd.get(), tmp_unitree_data.get());

        // Even though DJI motors receive proportional effort commands,
        // the mechanical reduction can be negative, which means that the
        // motor direction is inverted. In this case, the effort must be negated.
        // if (joint_motors_info_[i].mechanical_reduction < 0) {
        //     effort = -effort;
        // }

        // Write the command to the motor network
        // dji_motor_network_->write(i, effort);
    // }

    // Some motor network implementations require a separate tx() call
    

    return hardware_interface::return_type::OK;
}

} // namespace meta_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_hardware::MetaRobotUnitreeMotorInterface,
                       hardware_interface::SystemInterface)
