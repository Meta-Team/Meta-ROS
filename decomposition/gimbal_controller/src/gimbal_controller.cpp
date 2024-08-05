#include "gimbal_controller/gimbal_controller.hpp"

#include <control_toolbox/pid_ros.hpp>
#include <limits>
#include <memory>
#include <string>
#include <tf2/tf2/LinearMath/Matrix3x3.h>
#include <tf2/tf2/LinearMath/Quaternion.h>
#include <vector>

#include "angles/angles.h"
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
    gimbal_controller::GimbalController::ControllerReferenceMsg;
using ControllerFeedbackMsg =
    gimbal_controller::GimbalController::ControllerFeedbackMsg;

// called from RT control loop
void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & /*node*/) {
    msg->yaw = std::numeric_limits<double>::quiet_NaN();
    msg->pitch = std::numeric_limits<double>::quiet_NaN();
}

void reset_controller_feedback_msg(
    const std::shared_ptr<ControllerFeedbackMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) {
    msg->header.stamp = node->now();
    msg->orientation.x = std::numeric_limits<double>::quiet_NaN();
    msg->orientation.y = std::numeric_limits<double>::quiet_NaN();
    msg->orientation.z = std::numeric_limits<double>::quiet_NaN();
    msg->orientation.w = std::numeric_limits<double>::quiet_NaN();
    msg->angular_velocity.x = std::numeric_limits<double>::quiet_NaN();
    msg->angular_velocity.y = std::numeric_limits<double>::quiet_NaN();
    msg->angular_velocity.z = std::numeric_limits<double>::quiet_NaN();
    msg->linear_acceleration.x = std::numeric_limits<double>::quiet_NaN();
    msg->linear_acceleration.y = std::numeric_limits<double>::quiet_NaN();
    msg->linear_acceleration.z = std::numeric_limits<double>::quiet_NaN();
}

} // namespace

namespace gimbal_controller {
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
// using hardware_interface::HW_IF_VELOCITY;

GimbalController::GimbalController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn GimbalController::on_init() {

    try {
        param_listener_ =
            std::make_shared<gimbal_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during controller's init with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    // Allocate gimbal roles and PIDs
    gimbal_roles_.resize(params_.gimbal_joints.size());
    pos2vel_pids_.resize(params_.gimbal_joints.size());
    vel2eff_pids_.resize(params_.gimbal_joints.size());
    for (size_t i = 0; i < params_.gimbal_joints.size(); ++i) {
        if (params_.gimbal_roles[i] == "yaw") {
            gimbal_roles_[i] = gimbal_controller::gimbal_role::YAW;
        } else {
            gimbal_roles_[i] = gimbal_controller::gimbal_role::PITCH;
        }

        pos2vel_pids_[i] = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains." + params_.gimbal_joints[i] + "_pos2vel", true);

        if (!pos2vel_pids_[i]->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Failed to initialize PID for joint %s",
                         params_.gimbal_joints[i].c_str());
            return controller_interface::CallbackReturn::FAILURE;
        }

        vel2eff_pids_[i] = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains." + params_.gimbal_joints[i] + "_vel2eff", true);

        if (!vel2eff_pids_[i]->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Failed to initialize PID for joint %s",
                         params_.gimbal_joints[i].c_str());
            return controller_interface::CallbackReturn::FAILURE;
        }
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&GimbalController::reference_callback, this,
                  std::placeholders::_1));

    std::shared_ptr<ControllerReferenceMsg> msg =
        std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);

    // Feedback Subscriber
    feedback_subscriber_ =
        get_node()->create_subscription<ControllerFeedbackMsg>(
            params_.imu_topics[0], subscribers_qos,
            std::bind(&GimbalController::feedback_callback, this,
                      std::placeholders::_1));

    auto feedback_msg = std::make_shared<ControllerFeedbackMsg>();
    reset_controller_feedback_msg(feedback_msg, get_node());
    input_feedback_.writeFromNonRT(feedback_msg);

    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/controller_state", rclcpp::SystemDefaultsQoS());
        state_publisher_ =
            std::make_unique<ControllerStatePublisher>(s_publisher_);
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during publisher creation at configure "
                     "stage with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    state_publisher_->lock();
    state_publisher_->msg_.dof_states.resize(params_.gimbal_joints.size() * 2);
    for (size_t i = 0; i < params_.gimbal_joints.size(); ++i) {
        state_publisher_->msg_.dof_states[i].name =
            params_.gimbal_joints[i] + "_pos2vel";
        state_publisher_->msg_.dof_states[i + params_.gimbal_joints.size()]
            .name = params_.gimbal_joints[i] + "_vel2eff";
    }
    state_publisher_->unlock();

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(params_.gimbal_joints.size());
    for (const auto &joint : params_.gimbal_joints) {
        command_interfaces_config.names.push_back(joint + "/" + HW_IF_EFFORT);
    }

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
GimbalController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    // IMU feedback comes from ROS2 topic

    return state_interfaces_config;
}

void GimbalController::reference_callback(
    const std::shared_ptr<ControllerReferenceMsg> msg) {
    input_ref_.writeFromNonRT(msg);
}

void GimbalController::feedback_callback(
    const std::shared_ptr<ControllerFeedbackMsg> msg) {
    input_feedback_.writeFromNonRT(msg);
}

std::vector<hardware_interface::CommandInterface>
GimbalController::on_export_reference_interfaces() {
    reference_interfaces_.resize(2, std::numeric_limits<double>::quiet_NaN());

    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(reference_interfaces_.size());

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("yaw/") + HW_IF_POSITION,
                                      &reference_interfaces_[0]);

    reference_interfaces.emplace_back(get_node()->get_name(),
                                      std::string("pitch/") + HW_IF_POSITION,
                                      &reference_interfaces_[1]);

    return reference_interfaces;
}

bool GimbalController::on_set_chained_mode(bool chained_mode) {
    // Always accept switch to/from chained mode
    return true || chained_mode;
}

controller_interface::CallbackReturn GimbalController::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
    reset_controller_feedback_msg(*(input_feedback_.readFromRT()), get_node());

    reference_interfaces_.assign(reference_interfaces_.size(),
                                 std::numeric_limits<double>::quiet_NaN());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GimbalController::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
GimbalController::update_reference_from_subscribers() {
    auto current_ref =
        *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                    // immediately to prevent dangling

    if (!std::isnan(current_ref->yaw) && !std::isnan(current_ref->pitch)) {
        reference_interfaces_[0] = current_ref->yaw;
        reference_interfaces_[1] = current_ref->pitch;

        current_ref->yaw = std::numeric_limits<double>::quiet_NaN();
        current_ref->pitch = std::numeric_limits<double>::quiet_NaN();
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type
GimbalController::update_and_write_commands(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) {
    // Collect IMU feedback
    auto current_feedback = *(input_feedback_.readFromRT());

    double fb_yaw;
    double fb_pitch;
    double fb_roll;
    auto q = tf2::Quaternion(
        current_feedback->orientation.x, current_feedback->orientation.y,
        current_feedback->orientation.z, current_feedback->orientation.w);
    tf2::Matrix3x3(q).getRPY(fb_roll, fb_pitch, fb_yaw);

    double fb_yaw_vel = -current_feedback->angular_velocity.z;
    double fb_pitch_vel = -current_feedback->angular_velocity.y;

    // Update commands
    size_t num_joints = params_.gimbal_joints.size();
    std::vector<double> pos_refrences(num_joints);
    std::vector<double> pos_feedbacks(num_joints);
    std::vector<double> pos_errors(num_joints);
    std::vector<double> vel_references(num_joints);
    std::vector<double> vel_feedbacks(num_joints);
    std::vector<double> vel_errors(num_joints);
    std::vector<double> eff_commands(num_joints);

    if (!std::isnan(reference_interfaces_[0]) &&
        !std::isnan(reference_interfaces_[1]) && !std::isnan(fb_yaw) &&
        !std::isnan(fb_pitch) && !std::isnan(fb_roll) &&
        !std::isnan(fb_yaw_vel) && !std::isnan(fb_pitch_vel)) {

        for (size_t i = 0; i < command_interfaces_.size(); ++i) {
            pos_refrences[i] =
                (gimbal_roles_[i] == gimbal_controller::gimbal_role::YAW)
                    ? reference_interfaces_[0]
                    : -reference_interfaces_[1];
            pos_feedbacks[i] =
                (gimbal_roles_[i] == gimbal_controller::gimbal_role::YAW)
                    ? fb_yaw
                    : fb_pitch;
            vel_feedbacks[i] =
                (gimbal_roles_[i] == gimbal_controller::gimbal_role::YAW)
                    ? fb_yaw_vel
                    : fb_pitch_vel;

            // Position (IMU) to velocity (IMU) PID
            pos_errors[i] = angles::shortest_angular_distance(pos_feedbacks[i],
                                                              pos_refrences[i]);
            vel_references[i] =
                pos2vel_pids_[i]->computeCommand(pos_errors[i], period);

            // Velocity (IMU) to effort (motor) PID
            vel_errors[i] = vel_references[i] - vel_feedbacks[i];
            eff_commands[i] =
                vel2eff_pids_[i]->computeCommand(vel_errors[i], period);
            command_interfaces_[i].set_value(eff_commands[i]);
        }
    }

    // Publish state
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;

        for (size_t i = 0; i < num_joints; ++i) {
            state_publisher_->msg_.dof_states[i].reference = pos_refrences[i];
            state_publisher_->msg_.dof_states[i].feedback = pos_feedbacks[i];
            state_publisher_->msg_.dof_states[i].error = pos_errors[i];
            state_publisher_->msg_.dof_states[i].time_step = period.seconds();
            state_publisher_->msg_.dof_states[i].output = vel_references[i];
        }

        for (size_t i = num_joints; i < 2 * num_joints; ++i) {
            state_publisher_->msg_.dof_states[i].reference =
                vel_references[i - num_joints];
            state_publisher_->msg_.dof_states[i].feedback =
                vel_feedbacks[i - num_joints];
            state_publisher_->msg_.dof_states[i].error =
                vel_errors[i - num_joints];
            state_publisher_->msg_.dof_states[i].time_step = period.seconds();
            state_publisher_->msg_.dof_states[i].output =
                eff_commands[i - num_joints];
        }

        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalController,
                       controller_interface::ChainableControllerInterface)
