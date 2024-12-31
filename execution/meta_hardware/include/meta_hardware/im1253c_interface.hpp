#ifndef META_HARDWARE__IM1253C_INTERFACE_HPP_
#define META_HARDWARE__IM1253C_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "meta_hardware/modbus_rtu_driver/modbus_rtu_driver.hpp"
#include "meta_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace meta_hardware {
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

class MetaRobotIm1253cManager : public hardware_interface::SystemInterface {
  public:
    ~MetaRobotIm1253cManager() override;
    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
    on_init(const hardware_interface::HardwareInfo &info) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::StateInterface>
    export_state_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    std::vector<hardware_interface::CommandInterface>
    export_command_interfaces() override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type
    read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
    hardware_interface::return_type
    write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
  private:
    std::vector<std::unique_ptr<ModbusRtuDriver>> device_;
    class SensorInterfaceData {
      public:
        double state_voltage = NaN;
        double state_current = NaN;
        double state_power = NaN;
    };
    std::vector<SensorInterfaceData> sensor_interface_data_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__IM1253C_INTERFACE_HPP_
