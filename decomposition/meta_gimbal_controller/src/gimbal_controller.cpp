#include "gimbal_controller/gimbal_controller.hpp"

#include <control_toolbox/pid_ros.hpp>
#include <limits>
#include <memory>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

using ControllerReferenceMsg =
    gimbal_controller::GimbalController::ControllerReferenceMsg;
using ControllerFeedbackMsg = gimbal_controller::GimbalController::ControllerFeedbackMsg;

void reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & /*node*/) {
    msg->yaw = NaN;
    msg->pitch = NaN;
}

void reset_controller_feedback_msg(
    const std::shared_ptr<ControllerFeedbackMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) {
    msg->header.stamp = node->now();
    msg->orientation.x = NaN;
    msg->orientation.y = NaN;
    msg->orientation.z = NaN;
    msg->orientation.w = NaN;
    msg->angular_velocity.x = NaN;
    msg->angular_velocity.y = NaN;
    msg->angular_velocity.z = NaN;
    msg->linear_acceleration.x = NaN;
    msg->linear_acceleration.y = NaN;
    msg->linear_acceleration.z = NaN;
}

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;

namespace gimbal_controller {

controller_interface::CallbackReturn GimbalController::on_init() {

    try {
        param_listener_ = std::make_shared<gimbal_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during controller's init with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();

    // Initialize PIDs
    yaw_pos2vel_pid_ = std::make_shared<control_toolbox::PidROS>(
        get_node(), "gains." + params_.yaw_gimbal_joint.name + "_pos2vel", true);
    pitch_pos2vel_pid_ = std::make_shared<control_toolbox::PidROS>(
        get_node(), "gains." + params_.pitch_gimbal_joint.name + "_pos2vel", true);

    if (!yaw_pos2vel_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for pos2vel");
        return controller_interface::CallbackReturn::FAILURE;
    }
    if (!pitch_pos2vel_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for pos2vel");
        return controller_interface::CallbackReturn::FAILURE;
    }

    yaw_vel2eff_pid_ = std::make_shared<control_toolbox::PidROS>(
        get_node(), "gains." + params_.yaw_gimbal_joint.name + "_vel2eff", true);
    pitch_vel2eff_pid_ = std::make_shared<control_toolbox::PidROS>(
        get_node(), "gains." + params_.pitch_gimbal_joint.name + "_vel2eff", true);

    if (!yaw_vel2eff_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for vel2eff");
        return controller_interface::CallbackReturn::FAILURE;
    }
    if (!pitch_vel2eff_pid_->initPid()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for vel2eff");
        return controller_interface::CallbackReturn::FAILURE;
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&GimbalController::reference_callback, this, std::placeholders::_1));

    auto msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);

    // Feedback Subscriber
    feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
        params_.imu_topic, subscribers_qos,
        std::bind(&GimbalController::feedback_callback, this, std::placeholders::_1));

    auto feedback_msg = std::make_shared<ControllerFeedbackMsg>();
    reset_controller_feedback_msg(feedback_msg, get_node());
    input_feedback_.writeFromNonRT(feedback_msg);
    try {
        // State publisher
        s_publisher_ = get_node()->create_publisher<ControllerStateMsg>(
            "~/controller_state", rclcpp::SystemDefaultsQoS());
        state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during publisher creation at configure "
                     "stage with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    state_publisher_->lock();
    state_publisher_->msg_.dof_states.resize(4);
    state_publisher_->msg_.dof_states[0].name = "yaw_pos2vel";
    state_publisher_->msg_.dof_states[1].name = "pitch_pos2vel";
    state_publisher_->msg_.dof_states[2].name = "yaw_vel2eff";
    state_publisher_->msg_.dof_states[3].name = "pitch_vel2eff";
    state_publisher_->unlock();

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(2);
    command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" +
                                              HW_IF_EFFORT);
    command_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" +
                                              HW_IF_EFFORT);

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
    reference_interfaces_.resize(2, NaN);

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

controller_interface::CallbackReturn
GimbalController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
    reset_controller_feedback_msg(*(input_feedback_.readFromRT()), get_node());

    reference_interfaces_.assign(reference_interfaces_.size(), NaN);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalController::update_reference_from_subscribers() {
    auto current_ref = *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                                   // immediately to prevent dangling

    if (!std::isnan(current_ref->yaw) && !std::isnan(current_ref->pitch)) {
        reference_interfaces_[0] = current_ref->yaw;
        reference_interfaces_[1] = current_ref->pitch;

        current_ref->yaw = NaN;
        current_ref->pitch = NaN;
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type
GimbalController::update_and_write_commands(const rclcpp::Time &time,
                                            const rclcpp::Duration &period) {
    // Collect IMU feedback
    auto current_feedback = *(input_feedback_.readFromRT());

    double yaw_pos_fb, pitch_pos_fb, roll_pos_fb;
    tf2::Quaternion q_imu;
    fromMsg(current_feedback->orientation, q_imu);
    tf2::Matrix3x3(q_imu).getRPY(roll_pos_fb, pitch_pos_fb, yaw_pos_fb);

    double yaw_vel_fb = current_feedback->angular_velocity.z;
    double pitch_vel_fb = current_feedback->angular_velocity.y;

    double yaw_pos_ref = NaN, pitch_pos_ref = NaN;
    double yaw_pos_err = NaN, pitch_pos_err = NaN;
    double yaw_vel_ref = NaN, pitch_vel_ref = NaN;
    double yaw_vel_err = NaN, pitch_vel_err = NaN;
    double yaw_eff_cmd = NaN, pitch_eff_cmd = NaN;

    // Calculate commands
    if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) &&
        !std::isnan(yaw_pos_fb) && !std::isnan(pitch_pos_fb) &&
        !std::isnan(roll_pos_fb) && !std::isnan(yaw_vel_fb) &&
        !std::isnan(pitch_vel_fb)) {
        if (params_.yaw_gimbal_joint.enable) {
            // Yaw Position (IMU) to velocity (IMU) PID
            yaw_pos_ref = reference_interfaces_[0];
            yaw_pos_err = angles::shortest_angular_distance(yaw_pos_fb, yaw_pos_ref);
            yaw_vel_ref = yaw_pos2vel_pid_->computeCommand(yaw_pos_err, period);
            // Yaw Velocity (IMU) to effort (motor) PID
            yaw_vel_err = yaw_vel_ref - yaw_vel_fb;
            yaw_eff_cmd = yaw_vel2eff_pid_->computeCommand(yaw_vel_err, period);
            command_interfaces_[0].set_value(yaw_eff_cmd);
        }

        if (params_.pitch_gimbal_joint.enable) {
            // Pitch Position (IMU) to velocity (IMU) PID
            pitch_pos_ref = -reference_interfaces_[1];
            pitch_pos_err =
                angles::shortest_angular_distance(pitch_pos_fb, pitch_pos_ref);
            pitch_vel_ref = pitch_pos2vel_pid_->computeCommand(pitch_pos_err, period);
            // Pitch Velocity (IMU) to effort (motor) PID
            pitch_vel_err = pitch_vel_ref - pitch_vel_fb;
            pitch_eff_cmd = pitch_vel2eff_pid_->computeCommand(pitch_vel_err, period);
            command_interfaces_[1].set_value(pitch_eff_cmd);
        }
    }

    // Publish state
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;
        if (params_.yaw_gimbal_joint.enable) {
            state_publisher_->msg_.dof_states[0].reference = yaw_pos_ref;
            state_publisher_->msg_.dof_states[0].feedback = yaw_pos_fb;
            state_publisher_->msg_.dof_states[0].error = yaw_pos_err;
            state_publisher_->msg_.dof_states[0].time_step = period.seconds();
            state_publisher_->msg_.dof_states[0].output = yaw_vel_ref;

            state_publisher_->msg_.dof_states[1].reference = pitch_pos_ref;
            state_publisher_->msg_.dof_states[1].feedback = pitch_pos_fb;
            state_publisher_->msg_.dof_states[1].error = pitch_pos_err;
            state_publisher_->msg_.dof_states[1].time_step = period.seconds();
            state_publisher_->msg_.dof_states[1].output = pitch_vel_ref;
        }

        if (params_.pitch_gimbal_joint.enable) {
            state_publisher_->msg_.dof_states[2].reference = yaw_vel_ref;
            state_publisher_->msg_.dof_states[2].feedback = yaw_vel_fb;
            state_publisher_->msg_.dof_states[2].error = yaw_vel_err;
            state_publisher_->msg_.dof_states[2].time_step = period.seconds();
            state_publisher_->msg_.dof_states[2].output = yaw_eff_cmd;

            state_publisher_->msg_.dof_states[3].reference = pitch_vel_ref;
            state_publisher_->msg_.dof_states[3].feedback = pitch_vel_fb;
            state_publisher_->msg_.dof_states[3].error = pitch_vel_err;
            state_publisher_->msg_.dof_states[3].time_step = period.seconds();
            state_publisher_->msg_.dof_states[3].output = pitch_eff_cmd;
        }

        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalController,
                       controller_interface::ChainableControllerInterface)
