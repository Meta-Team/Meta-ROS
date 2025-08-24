#ifndef META_HARDWARE__UNITREE_MOTOR_INTERFACE_HPP_
#define META_HARDWARE__UNITREE_MOTOR_INTERFACE_HPP_

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "meta_hardware/motor_network/dji_motor_network.hpp"
#include "meta_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "unitree_sdk/unitreeMotor.h"
#include "unitree_sdk/SerialPort.h"

namespace meta_hardware {
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

class MetaRobotUnitreeMotorInterface : public hardware_interface::SystemInterface {
  public:
    ~MetaRobotUnitreeMotorInterface() override;

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
    class JointInterfaceData {
      public:
        double command_position = NaN;
        double command_velocity = NaN;
        
        double state_position = NaN;
        double state_velocity = NaN;
        double state_effort = NaN;
    };
    std::vector<JointInterfaceData> joint_interface_data_;

    class JointMotorInfo {
      public:
        std::string joint_name;
        double mechanical_reduction;
        double offset;
    };
    std::vector<JointMotorInfo>
        joint_motors_info_; // local cache of joint motor info

    std::unique_ptr<SerialPort> tmp_unitree_serialport;
    std::unique_ptr<MotorCmd> tmp_unitree_cmd;
    std::unique_ptr<MotorData> tmp_unitree_data;
};

}
#endif // META_HARDWARE__UNITREE_MOTOR_INTERFACE_HPP_
