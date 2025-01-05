#include "meta_chassis_controller/omni_chassis_controller.hpp"

#include <Eigen/src/Core/Matrix.h>
#include <behavior_interface/msg/detail/chassis__struct.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

using ControllerReferenceMsg =
    meta_chassis_controller::OmniChassisController::ControllerReferenceMsg;

void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) {
    msg->header.stamp = node->now();
    msg->twist.angular.x = NaN;
    msg->twist.angular.y = NaN;
    msg->twist.angular.z = NaN;
    msg->twist.linear.x = NaN;
    msg->twist.linear.y = NaN;
    msg->twist.linear.z = NaN;
}

void reset_chassis_cmd_msg(const std::shared_ptr<behavior_interface::msg::Chassis> &msg) {
    msg->mode = behavior_interface::msg::Chassis::CHASSIS;
    msg->max_power = NaN;
}

namespace meta_chassis_controller {
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

OmniChassisController::OmniChassisController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn OmniChassisController::on_init() {

    try {
        param_listener_ =
            std::make_shared<omni_chassis_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        fprintf(stderr, "Exception thrown during controller's init with message: %s \n",
                e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmniChassisController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    omni_wheel_kinematics_ = std::make_unique<OmniWheelKinematics>(
        params_.omni_wheel_forward_angles, params_.omni_wheel_center_x,
        params_.omni_wheel_center_y, params_.omni_wheel_sliding_angles,
        params_.omni_wheel_radius);

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
    twist_sub_ = get_node()->create_subscription<ControllerReferenceMsgUnstamped>(
        "~/reference", subscribers_qos,
        std::bind(&OmniChassisController::reference_callback, this,
                  std::placeholders::_1));

    subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.reliable();
    // subscribers_qos.transient_local();

    chassis_sub_ = get_node()->create_subscription<behavior_interface::msg::Chassis>(
        "/chassis_cmd", subscribers_qos,
        [this](behavior_interface::msg::Chassis::UniquePtr msg) {
            chassis_buf_.writeFromNonRT(std::move(msg));
        });

    {
        auto msg = std::make_shared<ControllerReferenceMsg>();
        reset_controller_reference_msg(msg, get_node());
        ref_buf_.writeFromNonRT(msg);
    }

    {
        auto msg = std::make_shared<behavior_interface::msg::Chassis>();
        reset_chassis_cmd_msg(msg);
        chassis_buf_.writeFromNonRT(msg);
    }

    follow_pid_ =
        std::make_shared<control_toolbox::PidROS>(get_node(), "follow_pid_gains", true);

    if (!follow_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize chassis follow PID");
        return controller_interface::CallbackReturn::FAILURE;
    }

    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/state", rclcpp::SystemDefaultsQoS());
        state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
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

    // Joint position state of yaw gimbal is required
    state_interfaces_config.names.reserve(1);
    state_interfaces_config.names.push_back(params_.yaw_gimbal_joint + "/" +
                                            HW_IF_POSITION);

    return state_interfaces_config;
}

void OmniChassisController::reference_callback(
    ControllerReferenceMsgUnstamped::UniquePtr msg) {
    auto stamped_msg = std::make_shared<ControllerReferenceMsg>();
    stamped_msg->header.stamp = get_node()->now();
    stamped_msg->twist = *msg;
    ref_buf_.writeFromNonRT(stamped_msg);
}

std::vector<hardware_interface::CommandInterface>
OmniChassisController::on_export_reference_interfaces() {
    reference_interfaces_.resize(3, NaN);

    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("linear/x/") + HW_IF_VELOCITY,
                                      &reference_interfaces_[0]);

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("linear/y/") + HW_IF_VELOCITY,
                                      &reference_interfaces_[1]);

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("angular/z/") + HW_IF_VELOCITY,
                                      &reference_interfaces_[2]);

    return reference_interfaces;
}

bool OmniChassisController::on_set_chained_mode(bool chained_mode) {
    // Always accept switch to/from chained mode
    return true || chained_mode;
}

controller_interface::CallbackReturn
OmniChassisController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(ref_buf_.readFromRT()), get_node());
    reset_chassis_cmd_msg(*(chassis_buf_.readFromRT()));

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
OmniChassisController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    for (size_t i = 0; i < command_interfaces_.size(); ++i) {
        command_interfaces_[i].set_value(NaN);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
OmniChassisController::update_reference_from_subscribers() {
    auto cur_ref = *(ref_buf_.readFromRT());

    if (const auto command_age = get_node()->now() - cur_ref->header.stamp;
        command_age <= ref_timeout_ ||
        ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
        if (!std::isnan(cur_ref->twist.linear.x) &&
            !std::isnan(cur_ref->twist.linear.y) &&
            !std::isnan(cur_ref->twist.angular.z)) {
            reference_interfaces_[0] = cur_ref->twist.linear.x;
            reference_interfaces_[1] = cur_ref->twist.linear.y;
            reference_interfaces_[2] = cur_ref->twist.angular.z;
        }
    } else {
        reference_interfaces_[0] = 0.0;
        reference_interfaces_[1] = 0.0;
        reference_interfaces_[2] = 0.0;
    }
    return controller_interface::return_type::OK;
}

controller_interface::return_type
OmniChassisController::update_and_write_commands(const rclcpp::Time &time,
                                                 const rclcpp::Duration &period) {
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
    }

    auto cur_chassis = *(chassis_buf_.readFromRT());

    if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) &&
        !std::isnan(reference_interfaces_[2])) {
        Eigen::Vector3d twist;
        twist << reference_interfaces_[0], reference_interfaces_[1],
            reference_interfaces_[2];

        switch (cur_chassis->mode) {
        case behavior_interface::msg::Chassis::CHASSIS:
            break;
        case behavior_interface::msg::Chassis::GIMBAL:
            gimbal_mode(twist);
            break;
        case behavior_interface::msg::Chassis::CHASSIS_FOLLOW:
            chassis_follow_mode(twist, time, period);
            break;
        case behavior_interface::msg::Chassis::GYRO:
            gyro_mode(twist);
            break;
        default:
            RCLCPP_ERROR(get_node()->get_logger(), "Unkown chassis mode: %d",
                         cur_chassis->mode);
            return controller_interface::return_type::ERROR;
        }

        Eigen::VectorXd wheel_vels = omni_wheel_kinematics_->inverse(twist);
        for (size_t i = 0; i < command_interfaces_.size(); i++) {
            command_interfaces_[i].set_value(wheel_vels[static_cast<Eigen::Index>(i)]);
        }
    }

    return controller_interface::return_type::OK;
}

void OmniChassisController::chassis_follow_mode(Eigen::Vector3d &twist,
                                                const rclcpp::Time &time,
                                                const rclcpp::Duration &period) {
    double yaw_gimbal_joint_pos = state_interfaces_[0].get_value();
    double error = -angles::shortest_angular_distance(yaw_gimbal_joint_pos,
                                                      params_.follow_pid_target);
    twist[2] = follow_pid_->computeCommand(error, period);

    transform_twist_to_gimbal(twist, yaw_gimbal_joint_pos);

    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;
        state_publisher_->msg_.set_point = 0.0;
        state_publisher_->msg_.process_value = state_interfaces_[0].get_value();
        state_publisher_->msg_.error = error;
        state_publisher_->msg_.time_step = period.seconds();
        state_publisher_->msg_.command = twist[2];

        state_publisher_->unlockAndPublish();
    }
}

void OmniChassisController::gimbal_mode(Eigen::Vector3d &twist) {
    transform_twist_to_gimbal(twist, state_interfaces_[0].get_value());
}

void OmniChassisController::gyro_mode(Eigen::Vector3d &twist) {
    twist[2] = 4.0;
    transform_twist_to_gimbal(twist, state_interfaces_[0].get_value());
}

void OmniChassisController::transform_twist_to_gimbal(
    Eigen::Vector3d &twist, const double &yaw_gimbal_joint_pos) const {
    Eigen::MatrixXd rotation_mat(3, 3);
    rotation_mat << cos(yaw_gimbal_joint_pos), -sin(yaw_gimbal_joint_pos), 0,
        sin(yaw_gimbal_joint_pos), cos(yaw_gimbal_joint_pos), 0, 0, 0, 1;
    twist = rotation_mat * twist;
}

} // namespace meta_chassis_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(meta_chassis_controller::OmniChassisController,
                       controller_interface::ChainableControllerInterface)
