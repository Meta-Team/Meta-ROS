#include "meta_chassis_controller/omni_chassis_controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace { // utility

static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    1, // message queue depth
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

using ControllerReferenceMsg =
    meta_chassis_controller::OmniChassisController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) {
    msg->header.stamp = node->now();
    msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
    msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
    msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
    msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
    msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
}

} // namespace

namespace meta_chassis_controller {
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

OmniChassisController::OmniChassisController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn OmniChassisController::on_init() {
    // control_mode_.initRT(control_mode_type::CHASSIS);

    try {
        param_listener_ =
            std::make_shared<omni_chassis_controller::ParamListener>(
                get_node());
    } catch (const std::exception &e) {
        fprintf(stderr,
                "Exception thrown during controller's init with message: %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniChassisController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    omni_wheel_kinematics_ = std::make_unique<OmniWheelKinematics>(
        params_.omni_wheel_angles, params_.omni_wheel_distance,
        params_.omni_wheel_radius);

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
    ref_subscriber_ =
        get_node()->create_subscription<ControllerReferenceMsgUnstamped>(
            "~/reference", subscribers_qos,
            std::bind(&OmniChassisController::reference_callback, this,
                      std::placeholders::_1));

    std::shared_ptr<ControllerReferenceMsg> msg =
        std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);

    follow_pid_ = std::make_shared<control_toolbox::PidROS>(
        get_node(), "follow_pid_gains", true);

    if (!follow_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Failed to initialize chassis follow PID");
        return controller_interface::CallbackReturn::FAILURE;
    }

    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/state", rclcpp::SystemDefaultsQoS());
        state_publisher_ =
            std::make_unique<ControllerStatePublisher>(s_publisher_);
    } catch (const std::exception &e) {
        fprintf(stderr,
                "Exception thrown during publisher creation at configure stage "
                "with message : %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    state_publisher_->lock();
    state_publisher_->unlock();

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
OmniChassisController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(params_.omni_wheel_joints.size());
    for (const auto &joint : params_.omni_wheel_joints) {
        command_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
OmniChassisController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    // Joint position state of yaw gimbal is required for GIMBAL or
    // CHASSIS_FOLLOW_GIMBAL control mode
    if (params_.control_mode == static_cast<int>(control_mode_type::GIMBAL) ||
        params_.control_mode ==
            static_cast<int>(control_mode_type::CHASSIS_FOLLOW_GIMBAL)) {
        state_interfaces_config.names.reserve(1);
        state_interfaces_config.names.push_back(params_.yaw_gimbal_joint + "/" +
                                                HW_IF_POSITION);
    }

    return state_interfaces_config;
}

void OmniChassisController::reference_callback(
    const std::shared_ptr<ControllerReferenceMsgUnstamped> msg) {
    auto stamped_msg = std::make_shared<ControllerReferenceMsg>();
    stamped_msg->header.stamp = get_node()->now();
    stamped_msg->twist = *msg;
    input_ref_.writeFromNonRT(stamped_msg);
}

std::vector<hardware_interface::CommandInterface>
OmniChassisController::on_export_reference_interfaces() {
    reference_interfaces_.resize(3, std::numeric_limits<double>::quiet_NaN());

    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());

    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), std::string("linear/x/") + HW_IF_VELOCITY,
        &reference_interfaces_[0]));

    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), std::string("linear/y/") + HW_IF_VELOCITY,
        &reference_interfaces_[1]));

    reference_interfaces.push_back(hardware_interface::CommandInterface(
        get_node()->get_name(), std::string("angular/z/") + HW_IF_VELOCITY,
        &reference_interfaces_[2]));

    return reference_interfaces;
}

bool OmniChassisController::on_set_chained_mode(bool chained_mode) {
    // Always accept switch to/from chained mode
    return true || chained_mode;
}

controller_interface::CallbackReturn OmniChassisController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniChassisController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(
            std::numeric_limits<double>::quiet_NaN());
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
OmniChassisController::update_reference_from_subscribers() {
    auto current_ref =
        *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                    // immediately to prevent dangling
    const auto command_age = get_node()->now() - current_ref->header.stamp;

    // RCLCPP_INFO(get_node()->get_logger(), "Current reference age: %f",
    // command_age.seconds());

    if (command_age <= ref_timeout_ ||
        ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
        if (!std::isnan(current_ref->twist.linear.x) &&
            !std::isnan(current_ref->twist.linear.y) &&
            !std::isnan(current_ref->twist.angular.z)) {
            reference_interfaces_[0] = current_ref->twist.linear.x;
            reference_interfaces_[1] = current_ref->twist.linear.y;
            reference_interfaces_[2] = current_ref->twist.angular.z;
        }
    } else {
        if (!std::isnan(current_ref->twist.linear.x) &&
            !std::isnan(current_ref->twist.linear.y) &&
            !std::isnan(current_ref->twist.angular.z)) {
            reference_interfaces_[0] = 0.0;
            reference_interfaces_[1] = 0.0;
            reference_interfaces_[2] = 0.0;
            current_ref->twist.linear.x =
                std::numeric_limits<double>::quiet_NaN();
            current_ref->twist.linear.y =
                std::numeric_limits<double>::quiet_NaN();
            current_ref->twist.angular.z =
                std::numeric_limits<double>::quiet_NaN();
        }
    }
    return controller_interface::return_type::OK;
}

controller_interface::return_type
OmniChassisController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }

    if (!std::isnan(reference_interfaces_[0]) &&
        !std::isnan(reference_interfaces_[1]) &&
        !std::isnan(reference_interfaces_[2])) {
        Eigen::Vector3d twist;
        twist << reference_interfaces_[0], reference_interfaces_[1],
            reference_interfaces_[2];
        if (params_.control_mode ==
            static_cast<int>(control_mode_type::CHASSIS_FOLLOW_GIMBAL)) {
            double current_motor_pos = state_interfaces_[0].get_value() - params_.yaw_gimbal_joint_offset;
            double error = current_motor_pos - params_.follow_pid_target;
            twist[2] = follow_pid_->computeCommand(error, period);
            if (state_publisher_ && state_publisher_->trylock()) {
                state_publisher_->msg_.header.stamp = time;
                state_publisher_->msg_.set_point = 0.0;
                state_publisher_->msg_.process_value =
                    state_interfaces_[0].get_value();
                state_publisher_->msg_.error = error;
                state_publisher_->msg_.time_step = period.seconds();
                state_publisher_->msg_.command = twist[2];

                state_publisher_->unlockAndPublish();
            }
        }
        if (params_.control_mode ==
                static_cast<int>(control_mode_type::GIMBAL) ||
            params_.control_mode ==
                static_cast<int>(control_mode_type::CHASSIS_FOLLOW_GIMBAL)) {
            Eigen::MatrixXd rotation_mat(3, 3);
            double yaw_gimbal_joint_pos = state_interfaces_[0].get_value() - params_.yaw_gimbal_joint_offset;
            rotation_mat << cos(yaw_gimbal_joint_pos),
                -sin(yaw_gimbal_joint_pos), 0, sin(yaw_gimbal_joint_pos),
                cos(yaw_gimbal_joint_pos), 0, 0, 0, 1;
            twist = rotation_mat * twist;
        }
        auto wheel_vels = omni_wheel_kinematics_->inverse(twist);
        for (size_t i = 0; i < command_interfaces_.size(); i++) {
            command_interfaces_[i].set_value(
                wheel_vels[static_cast<Eigen::Index>(i)]);
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace meta_chassis_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_chassis_controller::OmniChassisController,
                       controller_interface::ChainableControllerInterface)
