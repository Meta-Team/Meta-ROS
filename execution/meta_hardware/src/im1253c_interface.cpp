#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <ranges>
#include <string>
#include <system_error>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "meta_hardware/im1253c_interface.hpp"
#include "meta_hardware/modbus_rtu_driver/modbus_rtu_driver.hpp"
#include "meta_hardware/motor_network/mi_motor_network.hpp"
#include "rclcpp/rclcpp.hpp"


constexpr int DEVICE_ADDRESS = 0x01;
constexpr int FUNCTION_CODE_READ = 0x03;
constexpr int REGISTER_ADDRESS = 0x48;
constexpr int REGISTER_NUM = 3;  // 3 registers for voltage, current and power

namespace meta_hardware {

MetaRobotIm1253cManager::~MetaRobotIm1253cManager() = default;

hardware_interface::CallbackReturn
MetaRobotIm1253cManager::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }
    sensor_interface_data_.resize(info_.sensors.size());
    return CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MetaRobotIm1253cManager::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    for (size_t i = 0; i < info_.sensors.size(); ++i) {
        device_.emplace_back(std::make_unique<ModbusRtuDriver>(info_.hardware_parameters));
        device_[i]->set_command(DEVICE_ADDRESS, FUNCTION_CODE_READ, REGISTER_ADDRESS, REGISTER_NUM);
    }
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

    for (size_t i = 0; i < info_.sensors.size(); ++i) {
        const auto &sensors_state_interfaces = info_.sensors[i].state_interfaces;
        if (contains_interface(sensors_state_interfaces, "voltage")) {
            state_interfaces.emplace_back(info_.sensors[i].name, std::string("voltage"),
                                          &sensor_interface_data_[i].state_voltage);
        }
        if (contains_interface(sensors_state_interfaces, "current")) {
            state_interfaces.emplace_back(info_.sensors[i].name, std::string("current"),
                                          &sensor_interface_data_[i].state_current);
        }
        if (contains_interface(sensors_state_interfaces, "power")) {
            state_interfaces.emplace_back(info_.sensors[i].name, std::string("power"),
                                          &sensor_interface_data_[i].state_power);
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MetaRobotIm1253cManager::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
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
    for(size_t i = 0; i < info_.sensors.size(); i++){
        std::vector<int> sensor_data_raw = device_[i]->get_reg_data(); 
        sensor_interface_data_[i].state_voltage = static_cast<double>(sensor_data_raw[0])/100.0;
        sensor_interface_data_[i].state_current = static_cast<double>(sensor_data_raw[1])/100.0;
        sensor_interface_data_[i].state_power = static_cast<double>(sensor_data_raw[2]); 
        // std::cout << "Voltage: " << sensor_interface_data_[i].state_voltage << std::endl;
        // std::cout << "Current: " << sensor_interface_data_[i].state_current << std::endl;
        // std::cout << "Power: " << sensor_interface_data_[i].state_power << std::endl;
    }

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
