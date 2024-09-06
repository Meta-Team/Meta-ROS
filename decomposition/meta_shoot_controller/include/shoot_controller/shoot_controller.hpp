#ifndef SHOOT_CONTROLLER__SHOOT_CONTROLLER_HPP_
#define SHOOT_CONTROLLER__SHOOT_CONTROLLER_HPP_

#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <rclcpp/subscription.hpp>

#include "control_toolbox/pid_ros.hpp"
#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"


#include "control_msgs/msg/joint_controller_state.hpp"
#include "control_msgs/msg/multi_dof_state_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "behavior_interface/msg/shoot.hpp"
#include "meta_shoot_controller_parameters.hpp"



namespace shoot_controller {
    class ShootController : public controller_interface::ChainableControllerInterface {
    public:
        ShootController()=default;

        controller_interface::CallbackReturn 
        on_init() override;

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

    protected:
        // override methods from ChainableControllerInterface
        std::vector<hardware_interface::CommandInterface>
            on_export_reference_interfaces() override;
        bool on_set_chained_mode(bool chained_mode) override;
        

    private:
        using ControllerReferenceMsg = behavior_interface::msg::Shoot;
        using ControllerStateMsg = control_msgs::msg::MultiDOFStateStamped;
        using ControllerStatePublisher = realtime_tools::RealtimePublisher<ControllerStateMsg>;

        double friction_wheel_speed_;
        bool friction_wheel_on_;
        bool bullet_loader_on_;

        std::shared_ptr<control_toolbox::PidROS> fric1_vel_pid_;
        std::shared_ptr<control_toolbox::PidROS> fric2_vel_pid_;
        std::shared_ptr<control_toolbox::PidROS> load_vel_pid_;
        
        std::shared_ptr<shoot_controller::ParamListener> param_listener_;
        shoot_controller::Params params_;

        rclcpp::Publisher<ControllerStateMsg>::SharedPtr s_publisher_;
        std::unique_ptr<ControllerStatePublisher> state_publisher_;

        // Command subscribers
        rclcpp::Subscription<ControllerReferenceMsg>::SharedPtr ref_subscriber_;
        realtime_tools::RealtimeBuffer<std::shared_ptr<ControllerReferenceMsg>> input_ref_;   

        void reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg);
        void feedback_callback(const std::shared_ptr<hardware_interface::StateInterface> motor_feedback);

        void reset_controller_reference_msg(const std::shared_ptr<ControllerReferenceMsg> &msg,
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);

        

    };
}
#endif // SHOOT_CONTROLLER__SHOOT_CONTROLLER_HPP_ 