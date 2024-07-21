#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_

#include <control_toolbox/pid_ros.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <string>
#include <vector>

#include "controller_interface/chainable_controller_interface.hpp"
#include "gimbal_controller_parameters.hpp"
#include "gimbal_controller/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/duration.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"
#include "control_toolbox/pid_ros.hpp"

#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace gimbal_controller
{
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

enum class control_mode_type : std::uint8_t
{
  CHASSIS = 0,
  GIMBAL = 1,
};

enum class gimbal_role : std::uint8_t
{
  PITCH = 0,
  YAW = 1,
};

class GimbalController : public controller_interface::ChainableControllerInterface
{
public:
  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  GimbalController();

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_reference_from_subscribers() override;

  GIMBAL_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  using ControllerFeedbackMsg = sensor_msgs::msg::Imu;
  using ControllerReferenceMsg = geometry_msgs::msg::Vector3Stamped;
  using ControllerReferenceMsgUnstamped = geometry_msgs::msg::Vector3;
  using ControllerModeSrvType = std_srvs::srv::SetBool;
  using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

protected:
  std::shared_ptr<gimbal_controller::ParamListener> param_listener_;
  gimbal_controller::Params params_;

  std::vector<std::string> state_joints_;

  // Command subscribers
  rclcpp::Duration ref_timeout_ = rclcpp::Duration(0, 0);
  rclcpp::Subscription<ControllerReferenceMsgUnstamped>::SharedPtr ref_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

  // Feedback subscribers
  rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>> input_feedback_;

  std::vector<std::shared_ptr<control_toolbox::PidROS>> pids_;

  rclcpp::Service<ControllerModeSrvType>::SharedPtr set_slow_control_mode_service_;
  realtime_tools::RealtimeBuffer<control_mode_type> control_mode_;

  using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

  rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
  std::unique_ptr<ControllerStatePublisher> state_publisher_;

  // override methods from ChainableControllerInterface
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

  bool on_set_chained_mode(bool chained_mode) override;
  
  std::vector<gimbal_role> gimbal_roles_;

private:
  // callback for topic interface
  GIMBAL_CONTROLLER__VISIBILITY_LOCAL
  void reference_callback(const std::shared_ptr<ControllerReferenceMsgUnstamped> msg);
  void feedback_callback(const std::shared_ptr<ControllerFeedbackMsg> msg);
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_