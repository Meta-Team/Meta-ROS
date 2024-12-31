#ifndef POWER_LIMIT__POWER_LIMIT_HPP_
#define POWER_LIMIT__POWER_LIMIT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "device_interface/msg/motor_power.hpp"
// #include "controller_interface/chainable_controller_interface.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "power_limit_parameters.hpp"
// #include ""

#include "control_msgs/msg/joint_controller_state.hpp"

namespace power_limit {

enum class control_mode_type : std::uint8_t {
    CHASSIS = 0,
    GIMBAL = 1,
    CHASSIS_FOLLOW_GIMBAL = 2,
};

class PowerLimitController : public controller_interface::ControllerInterface {
  public:
    PowerLimitController();

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

    controller_interface::return_type
    update(const rclcpp::Time &time,
                              const rclcpp::Duration &period) override;

    // using ControllerStateMsg = control_msgs::msg::JointControllerState;
    using MotorPowerMsg = device_interface::msg::MotorPower;

  private:
    std::shared_ptr<power_limit::ParamListener> param_listener_;
    power_limit::Params params_;


    rclcpp::Duration ref_timeout_ = rclcpp::Duration(0, 0);
    // rclcpp::Subscription<ControllerReferenceMsgUnstamped>::SharedPtr twist_sub_;
    // realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> ref_buf_;

    // rclcpp::Subscription<behavior_interface::msg::Chassis>::SharedPtr chassis_sub_;
    // realtime_tools::RealtimeBuffer<std::shared_ptr<behavior_interface::msg::Chassis>>
    //     chassis_buf_;

    using MotorPowerPublisher =
        realtime_tools::RealtimePublisher<MotorPowerMsg>;

    rclcpp::Publisher<MotorPowerMsg>::SharedPtr s_publisher_;
    std::unique_ptr<MotorPowerPublisher> state_publisher_;

};

} // namespace power_limit

#endif // POWER_LIMIT__POWER_LIMIT_HPP_