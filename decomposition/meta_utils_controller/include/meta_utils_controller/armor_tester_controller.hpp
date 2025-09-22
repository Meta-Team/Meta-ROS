#ifndef ARMOR_TESTER_CONTROLLER_HPP_
#define ARMOR_TESTER_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <control_toolbox/pid_ros.hpp>
#include <rclcpp/subscription.hpp>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include <meta_utils_controller/armor_tester_controller_parameters.hpp>
#include "behavior_interface/msg/armor.hpp" // dji_vel, unitree_vel

namespace armor_tester_controller // namespace begin here
{

class ArmorTesterController : public controller_interface::ChainableControllerInterface
{
    public:
        ArmorTesterController() = default;

        // ControllerInterfaceBase and ChainableControllerInterface, a little strange
        ~ArmorTesterController() = default;

        // override method from ControllerInterfaceBase (done?)
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        // override method from ControllerInterfaceBase (done?)
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        // override method from ControllerInterfaceBase (done)
        controller_interface::CallbackReturn on_init() override;

        // override method from ControllerInterfaceBase (done)
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        // override method from ControllerInterfaceBase (done)
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        // override method from ControllerInterfaceBase (done)
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        // override method from ChainableControllerInterface
        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    protected:
        // override method from ChainableControllerInterface
        controller_interface::return_type update_reference_from_subscribers() override;

        // parameters
        armor_tester_controller::Params params_;
        std::shared_ptr<armor_tester_controller::ParamListener> param_listener_;

        // pid for dji motor
        std::shared_ptr<control_toolbox::PidROS> dji_pid_;

        // for subscriber
        rclcpp::Subscription<behavior_interface::msg::Armor>::SharedPtr vel_subscriber_ = nullptr;
        realtime_tools::RealtimeBuffer<std::shared_ptr<behavior_interface::msg::Armor>> vel_buffer_;

        // override method from ChainableControllerInterface (done)
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

    private:
        // callback function for subscriber (done)
        void velocity_callback(const std::shared_ptr<behavior_interface::msg::Armor> msg);
}; // class definition ends here

} // namespace ends here

#endif // ARMOR_TESTER_CONTROLLER_HPP_
