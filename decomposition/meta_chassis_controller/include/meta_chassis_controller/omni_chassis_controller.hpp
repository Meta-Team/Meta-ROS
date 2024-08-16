#ifndef OMNI_CHASSIS_CONTROLLER__OMNI_CHASSIS_CONTROLLER_HPP_
#define OMNI_CHASSIS_CONTROLLER__OMNI_CHASSIS_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behavior_interface/msg/chassis.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "meta_chassis_controller/omni_wheel_kinematics.hpp"
#include "omni_chassis_controller_parameters.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "control_msgs/msg/joint_controller_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace meta_chassis_controller {

enum class control_mode_type : std::uint8_t {
    CHASSIS = 0,
    GIMBAL = 1,
    CHASSIS_FOLLOW_GIMBAL = 2,
};

class OmniChassisController : public controller_interface::ChainableControllerInterface {
  public:
    OmniChassisController();

    controller_interface::CallbackReturn on_init() override;

    controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration
    state_interface_configuration() const override;

    controller_interface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update_reference_from_subscribers() override;

    controller_interface::return_type
    update_and_write_commands(const rclcpp::Time &time,
                              const rclcpp::Duration &period) override;

    using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
    using ControllerReferenceMsgUnstamped = geometry_msgs::msg::Twist;
    using ControllerStateMsg = control_msgs::msg::JointControllerState;

  private:
    std::shared_ptr<omni_chassis_controller::ParamListener> param_listener_;
    omni_chassis_controller::Params params_;

    std::shared_ptr<control_toolbox::PidROS> follow_pid_;

    rclcpp::Duration ref_timeout_ = rclcpp::Duration(0, 0);
    rclcpp::Subscription<ControllerReferenceMsgUnstamped>::SharedPtr twist_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> ref_buf_;

    rclcpp::Subscription<behavior_interface::msg::Chassis>::SharedPtr chassis_sub_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<behavior_interface::msg::Chassis>>
        chassis_buf_;

    using ControllerStatePublisher =
        realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    // Omni wheel kinematics
    std::unique_ptr<OmniWheelKinematics> omni_wheel_kinematics_;

    // override methods from ChainableControllerInterface
    std::vector<hardware_interface::CommandInterface>
    on_export_reference_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

    // Callbacks
    void reference_callback(ControllerReferenceMsgUnstamped::UniquePtr msg);

    void chassis_follow_mode(Eigen::Vector3d &twist, const rclcpp::Time &time,
                             const rclcpp::Duration &period);
    void gimbal_mode(Eigen::Vector3d &twist);
    void gyro_mode(Eigen::Vector3d &twist);

    void transform_twist_to_gimbal(Eigen::Vector3d &twist, const double &yaw_gimbal_joint_pos) const;
};

} // namespace meta_chassis_controller

#endif // OMNI_CHASSIS_CONTROLLER__OMNI_CHASSIS_CONTROLLER_HPP_
