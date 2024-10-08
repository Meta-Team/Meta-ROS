#ifndef META_HARDWARE__DM_MOTOR_INTERFACE_HPP_
#define META_HARDWARE__DM_MOTOR_INTERFACE_HPP_

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "meta_hardware/motor_network/dm_motor_network.hpp"
#include "meta_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace meta_hardware {
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

class MetaRobotDmMotorNetwork : public hardware_interface::SystemInterface {
  public:
    ~MetaRobotDmMotorNetwork() override;
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

    enum class DmMotorMode {
        MIT, // Dynamic mode with all three commands
        MIT_POS,
        MIT_VEL,
        MIT_EFF,
        MIT_POS_FF,
        MIT_VEL_FF,
        POSITION, // Position mode
        VELOCITY, // Velocity mode
    };

  private:
    class JointInterfaceData {
      public:
        double command_position = NaN;
        double command_velocity = NaN;
        double command_effort = NaN;
        double state_position = NaN;
        double state_velocity = NaN;
        double state_effort = NaN;
    };
    std::vector<JointInterfaceData> joint_interface_data_;

    class JointMotorInfo {
      public:
        std::string name;
        double mechanical_reduction;
        double offset;
        bool command_pos;
        bool command_vel;
        bool command_eff;
        DmMotorMode mode;
    };
    std::vector<JointMotorInfo>
        joint_motor_info_; // local cache of joint motor info

    MetaRobotDmMotorNetwork::DmMotorMode check_motor_mode(const std::string &mode,bool command_pos,
                                          bool command_vel, bool command_eff);

    std::unique_ptr<DmMotorNetwork> dm_motor_network_;
};

} // namespace meta_hardware

#endif // META_HARDWARE__DM_MOTOR_INTERFACE_HPP_
