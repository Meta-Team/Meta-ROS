#ifndef OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_
#define OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "omni_wheel_controller_parameters.hpp"
#include "omni_wheel_controller/visibility_control.h"
#include "omni_wheel_controller/omni_wheel_kinematics.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "control_toolbox/pid_ros.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "control_msgs/msg/joint_controller_state.hpp"

namespace omni_wheel_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

enum class control_mode_type : std::uint8_t
{
  CHASSIS = 0,
  GIMBAL = 1,
  CHASSIS_FOLLOW_GIMBAL = 2,
};

class OmniWheelController : public controller_interface::ChainableControllerInterface
{
public:
  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  OmniWheelController();

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  OMNI_WHEEL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerReferenceMsg = geometry_msgs::msg::TwistStamped;
  using ControllerReferenceMsgUnstamped = geometry_msgs::msg::Twist;
  using ControllerStateMsg = control_msgs::msg::JointControllerState;

protected:
  std::shared_ptr<omni_wheel_controller::ParamListener> param_listener_;
  omni_wheel_controller::Params params_;

  std::shared_ptr<control_toolbox::PidROS> follow_pid_;

  // Command subscribers and Controller State publisher
  rclcpp::Duration ref_timeout_ = rclcpp::Duration(0, 0);
  rclcpp::Subscription<ControllerReferenceMsgUnstamped>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // Omni wheel kinematics
  std::unique_ptr<OmniWheelKinematics> omni_wheel_kinematics_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;

private:
  // callback for topic interface
  OMNI_WHEEL_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsgUnstamped> msg);
};

}  // namespace omni_wheel_controller

#endif  // OMNI_WHEEL_CONTROLLER__OMNI_WHEEL_CONTROLLER_HPP_
