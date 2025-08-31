#include "gimbal_controller/gimbal_position_controller.hpp"

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
    gimbal_controller::GimbalPositionController::ControllerReferenceMsg;
using ControllerFeedbackMsg = gimbal_controller::GimbalPositionController::ControllerFeedbackMsg;



using hardware_interface::HW_IF_POSITION;

namespace gimbal_controller {

void GimbalPositionController::reset_controller_reference_msg(
    const std::shared_ptr<ControllerReferenceMsg> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & /*node*/) {
    msg->yaw = NaN;
    msg->pitch = NaN;
}

void GimbalPositionController::reset_controller_feedback_msg(
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
controller_interface::CallbackReturn GimbalPositionController::on_init() {

    try {
        param_listener_ = std::make_shared<gimbal_position_controller::ParamListener>(get_node());
    } catch (const std::exception &e) {
        std::cerr << "Exception thrown during controller's init with message: "
                  << e.what() << std::endl;
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalPositionController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
    params_ = param_listener_->get_params();


    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsg>(
        "~/reference", subscribers_qos,
        std::bind(&GimbalPositionController::reference_callback, this, std::placeholders::_1));

    auto msg = std::make_shared<ControllerReferenceMsg>();
    reset_controller_reference_msg(msg, get_node());
    input_ref_.writeFromNonRT(msg);

    // Feedback Subscriber
    feedback_subscriber_ = get_node()->create_subscription<ControllerFeedbackMsg>(
        params_.imu_topic, subscribers_qos,
        std::bind(&GimbalPositionController::feedback_callback, this, std::placeholders::_1));

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
    state_publisher_->msg_.dof_states.resize(2);
    state_publisher_->msg_.dof_states[0].name = "gimbal_yaw_diff";
    state_publisher_->msg_.dof_states[1].name = "gimbal_pitch_diff";
    state_publisher_->unlock();

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GimbalPositionController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    command_interfaces_config.names.reserve(2);
    command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);
    command_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);

    return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
GimbalPositionController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type =
        controller_interface::interface_configuration_type::INDIVIDUAL;

    state_interfaces_config.names.reserve(2);
    state_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);
    state_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" +
                                              HW_IF_POSITION);
    // IMU feedback comes from ROS2 topic

    return state_interfaces_config;
}

void GimbalPositionController::reference_callback(
    const std::shared_ptr<ControllerReferenceMsg> msg) {
    input_ref_.writeFromNonRT(msg);
}

void GimbalPositionController::feedback_callback(
    const std::shared_ptr<ControllerFeedbackMsg> msg) {
    input_feedback_.writeFromNonRT(msg);
}

std::vector<hardware_interface::CommandInterface>
GimbalPositionController::on_export_reference_interfaces() {
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

bool GimbalPositionController::on_set_chained_mode(bool chained_mode) {
    // Always accept switch to/from chained mode
    return true || chained_mode;
}

controller_interface::CallbackReturn
GimbalPositionController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
    // Set default value in command
    reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());
    reset_controller_feedback_msg(*(input_feedback_.readFromRT()), get_node());

    reference_interfaces_.assign(reference_interfaces_.size(), NaN);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
GimbalPositionController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GimbalPositionController::update_reference_from_subscribers() {
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
GimbalPositionController::update_and_write_commands(const rclcpp::Time &time,
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
    
    double yaw_enc_pos = state_interfaces_[0].get_value();
    double pitch_enc_pos = state_interfaces_[1].get_value();
    // Calculate commands
    if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) &&
        !std::isnan(yaw_pos_fb) && !std::isnan(pitch_pos_fb) &&
        !std::isnan(roll_pos_fb) && !std::isnan(yaw_vel_fb) &&
        !std::isnan(pitch_vel_fb)) {
        if (params_.yaw_gimbal_joint.enable) {
            // Yaw Position (IMU) to velocity (IMU) PID
            yaw_pos_ref = reference_interfaces_[0];
            yaw_pos_err = angles::shortest_angular_distance(yaw_pos_fb, yaw_pos_ref);
            command_interfaces_[0].set_value(yaw_enc_pos + yaw_pos_err);
        }

        if (params_.pitch_gimbal_joint.enable) {
            // Pitch Position (IMU) to velocity (IMU) PID
            // for cybergear negative up, for imu negative up, for reference it should be negative up too
            pitch_pos_ref = -reference_interfaces_[1];
            pitch_pos_err = angles::shortest_angular_distance(pitch_pos_fb, pitch_pos_ref);
            command_interfaces_[1].set_value(pitch_enc_pos + pitch_pos_err);
        }
    }
    // rclcpp::Logger tmp_logger = rclcpp::get_logger("gpc");
    // RCLCPP_INFO(tmp_logger, "pitch:(ref:%.2lf, err:%.2lf, fb:%.2lf, enc:%.2lf, cmd:%.2lf)", pitch_pos_ref, pitch_pos_err, pitch_pos_fb, pitch_enc_pos, pitch_enc_pos + pitch_pos_err);
    // Publish state
    if (state_publisher_ && state_publisher_->trylock()) {
        state_publisher_->msg_.header.stamp = time;
        if (params_.yaw_gimbal_joint.enable) {
            state_publisher_->msg_.dof_states[0].reference = yaw_pos_ref;
            state_publisher_->msg_.dof_states[0].feedback = yaw_pos_fb;
            state_publisher_->msg_.dof_states[0].error = yaw_pos_err;
            state_publisher_->msg_.dof_states[0].time_step = period.seconds();
            state_publisher_->msg_.dof_states[0].output = yaw_enc_pos + yaw_pos_err;
        }

        if (params_.pitch_gimbal_joint.enable) {
            state_publisher_->msg_.dof_states[1].reference = pitch_pos_ref;
            state_publisher_->msg_.dof_states[1].feedback = pitch_pos_fb;
            state_publisher_->msg_.dof_states[1].error = pitch_pos_err;
            state_publisher_->msg_.dof_states[1].time_step = period.seconds();
            state_publisher_->msg_.dof_states[1].output = pitch_enc_pos + pitch_pos_err;
        }

        state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace gimbal_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalPositionController,
                       controller_interface::ChainableControllerInterface)
