#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_

#include <control_toolbox/pid_ros.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>
#include <string>
#include <vector>

#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "meta_gimbal_controller_parameters.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_srvs/srv/set_bool.hpp"

#include "behavior_interface/msg/aim.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace gimbal_controller {
// name constants for state interfaces
static constexpr size_t STATE_MY_ITFS = 0;

// name constants for command interfaces
static constexpr size_t CMD_MY_ITFS = 0;

class GimbalController : public controller_interface::ChainableControllerInterface {
  public:
    GimbalController() = default;

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

    using ControllerFeedbackMsg = sensor_msgs::msg::Imu;
    using ControllerReferenceMsg = behavior_interface::msg::Aim;
    using ControllerModeSrvType = std_srvs::srv::SetBool;
    using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;

  protected:
    std::shared_ptr<gimbal_controller::ParamListener> param_listener_;
    gimbal_controller::Params params_;

    // Command subscribers
    rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;

    // Feedback subscribers
    rclcpp::Subscription<ControllerFeedbackMsg>::SharedPtr feedback_subscriber_ = nullptr;
    realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerFeedbackMsg>>
        input_feedback_;

    std::shared_ptr<control_toolbox::PidROS> yaw_pos2vel_pid_;
    std::shared_ptr<control_toolbox::PidROS> pitch_pos2vel_pid_;
    std::shared_ptr<control_toolbox::PidROS> yaw_vel2eff_pid_;
    std::shared_ptr<control_toolbox::PidROS> pitch_vel2eff_pid_;

    using ControllerStatePublisher =
        realtime_tools::RealtimePublisher<ControllerStateMsg>;

    rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
    std::unique_ptr<ControllerStatePublisher> state_publisher_;

    // override methods from ChainableControllerInterface
    std::vector<hardware_interface::CommandInterface>
    on_export_reference_interfaces() override;

    bool on_set_chained_mode(bool chained_mode) override;

  private:
    void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
    void feedback_callback(const std::shared_ptr<ControllerFeedbackMsg> msg);
};

} // namespace gimbal_controller

#endif // GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_HPP_
