#ifndef METAV_HARDWARE__HARDWARE_INTERFACE_HPP_
#define METAV_HARDWARE__HARDWARE_INTERFACE_HPP_

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "metav_hardware/motor_network/can_motor_network.hpp"
#include "metav_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace metav_hardware {
class MetavRobotHardwareInterface : public hardware_interface::SystemInterface {
  public:
    ~MetavRobotHardwareInterface() override;
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
        double command_position;
        double command_velocity;
        double command_effort;
        double state_position;
        double state_velocity;
        double state_effort;
    };
    std::vector<JointInterfaceData> joint_interface_data_;

    class JointMotorInfo {
      public:
        std::string name;
        std::string motor_vendor;
        std::string can_network_name;
        std::shared_ptr<CanMotorNetwork> can_motor_network;
        double mechanical_reduction;
        bool command_pos;
        bool command_vel;
        bool command_eff;
    };
    std::vector<JointMotorInfo>
        joint_motors_info_; // local cache of joint motor info

    // store all the CAN motor networks in std::map
    // [motor_vendor, can_network_name] -> can_motor_network
    std::map<
        std::string,
        std::map<std::string, std::shared_ptr<CanMotorNetwork>, std::less<>>,
        std::less<>>
        can_motor_networks_;
};

} // namespace metav_hardware

#endif // METAV_HARDWARE__HARDWARE_INTERFACE_HPP_
