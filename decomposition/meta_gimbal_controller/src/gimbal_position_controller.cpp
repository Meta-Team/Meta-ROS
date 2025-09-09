#include "gimbal_controller/gimbal_position_controller.hpp"

#include <control_toolbox/pid_ros.hpp>
#include <limits>
#include <memory>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include "angles/angles.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

using ControllerReferenceMsg = gimbal_controller::GimbalPositionController::ControllerReferenceMsg;


using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

namespace gimbal_controller
{
    void GimbalPositionController::reset_controller_reference_msg(
        const std::shared_ptr<ControllerReferenceMsg>& msg,
        const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& /*node*/)
    {
        msg->yaw = NaN;
        msg->pitch = NaN;
    }

    controller_interface::CallbackReturn GimbalPositionController::on_init()
    {

        try
        {
            param_listener_ = std::make_shared<gimbal_position_controller::ParamListener>(get_node());
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception thrown during controller's init with message: " << e.what() << std::endl;
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    GimbalPositionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        params_ = param_listener_->get_params();
        // Initialize PIDs
        yaw_pos2vel_pid_ = std::make_shared<control_toolbox::PidROS>(
            get_node(), "gains." + params_.yaw_gimbal_joint.name + "_pos2vel", true);

        if (!yaw_pos2vel_pid_->initPid())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for pos2vel");
            return controller_interface::CallbackReturn::FAILURE;
        }
        // init imu sensor by params
        imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(params_.imu_sensor);
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

        try
        {
            // State publisher
            s_publisher_ =
                get_node()->create_publisher<ControllerStateMsg>("~/controller_state", rclcpp::SystemDefaultsQoS());
            state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception thrown during publisher creation at configure "
                         "stage with message: "
                      << e.what() << std::endl;
            return controller_interface::CallbackReturn::ERROR;
        }

        state_publisher_->lock();
        state_publisher_->msg_.dof_states.resize(2);
        state_publisher_->msg_.dof_states[0].name = "yaw_pos2vel";
        state_publisher_->msg_.dof_states[1].name = "pitch_pos2pos_enc";
        state_publisher_->unlock();

        RCLCPP_INFO(get_node()->get_logger(), "configure successful");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration GimbalPositionController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interfaces_config;
        command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        command_interfaces_config.names.reserve(4);
        command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" + HW_IF_POSITION);
        command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" + HW_IF_VELOCITY);
        command_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" + HW_IF_EFFORT);
        command_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" + HW_IF_POSITION);

        return command_interfaces_config;
    }

    controller_interface::InterfaceConfiguration GimbalPositionController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interfaces_config;
        state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        state_interfaces_config.names.reserve(2);
        state_interfaces_config.names.push_back(params_.yaw_gimbal_joint.name + "/" + HW_IF_POSITION);
        state_interfaces_config.names.push_back(params_.pitch_gimbal_joint.name + "/" + HW_IF_POSITION);

        // IMU feedback comes from state interfaces
        auto imu_interfaces = imu_sensor_->get_state_interface_names();
        state_interfaces_config.names.insert(state_interfaces_config.names.end(), imu_interfaces.begin(),
                                             imu_interfaces.end());

        return state_interfaces_config;
    }

    void GimbalPositionController::reference_callback(const std::shared_ptr<ControllerReferenceMsg> msg)
    {
        input_ref_.writeFromNonRT(msg);
    }

    std::vector<hardware_interface::CommandInterface> GimbalPositionController::on_export_reference_interfaces()
    {
        reference_interfaces_.resize(2, NaN);

        std::vector<hardware_interface::CommandInterface> reference_interfaces;
        reference_interfaces.reserve(reference_interfaces_.size());

        reference_interfaces.emplace_back(get_node()->get_name(), std::string("yaw/") + HW_IF_POSITION,
                                          &reference_interfaces_[0]);

        reference_interfaces.emplace_back(get_node()->get_name(), std::string("pitch/") + HW_IF_POSITION,
                                          &reference_interfaces_[1]);

        return reference_interfaces;
    }

    bool GimbalPositionController::on_set_chained_mode(bool chained_mode)
    {
        // Always accept switch to/from chained mode
        return true || chained_mode;
    }

    controller_interface::CallbackReturn
    GimbalPositionController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Set default value in command
        reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

        reference_interfaces_.assign(reference_interfaces_.size(), NaN);
        imu_sensor_->assign_loaned_state_interfaces(state_interfaces_);
        // imu_sensor_->assign_loaned_state_interfaces(state_interfaces_); //do not &state_interfaces_[2]
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    GimbalPositionController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        imu_sensor_->release_interfaces();
        return controller_interface::CallbackReturn::SUCCESS;
    }

#if RCLCPP_VERSION_MAJOR >= 28 // ROS 2 Jazzy or later
    controller_interface::return_type
    GimbalPositionController::update_reference_from_subscribers(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
    {
#else
    controller_interface::return_type GimbalPositionController::update_reference_from_subscribers()
    {
#endif
        auto current_ref = *(input_ref_.readFromRT()); // A shared_ptr must be allocated
                                                       // immediately to prevent dangling

        if (!std::isnan(current_ref->yaw) && !std::isnan(current_ref->pitch))
        {
            reference_interfaces_[0] = current_ref->yaw;
            reference_interfaces_[1] = current_ref->pitch;

            current_ref->yaw = NaN;
            current_ref->pitch = NaN;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::return_type
    GimbalPositionController::update_and_write_commands(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // Collect IMU feedback from sensor state interfaces
        double yaw_pos_fb = NaN, pitch_pos_fb = NaN, roll_pos_fb = NaN;
        double yaw_vel_fb = NaN, pitch_vel_fb = NaN;

        std::array<double, 4> tmp_q = imu_sensor_->get_orientation();
        tf2::Quaternion q_imu(tmp_q[0], tmp_q[1], tmp_q[2], tmp_q[3]);
        tf2::Matrix3x3(q_imu).getRPY(roll_pos_fb, pitch_pos_fb, yaw_pos_fb);

        std::array<double, 3> tmp_angular_velocity = imu_sensor_->get_angular_velocity();
        yaw_vel_fb = tmp_angular_velocity[2]; // r, p, y: 0, 1, 2
        pitch_vel_fb = tmp_angular_velocity[1];
        // get reference
        double yaw_pos_ref = NaN, pitch_pos_ref = NaN;
        double yaw_pos_err = NaN, pitch_pos_err = NaN;
        double yaw_vel_ref = NaN;

        // double yaw_enc_pos = state_interfaces_[0].get_value();
        double pitch_enc_pos = state_interfaces_[1].get_value();
        // Calculate commands
        if (!std::isnan(reference_interfaces_[0]) && !std::isnan(reference_interfaces_[1]) && !std::isnan(yaw_pos_fb) &&
            !std::isnan(pitch_pos_fb) && !std::isnan(roll_pos_fb) && !std::isnan(yaw_vel_fb) &&
            !std::isnan(pitch_vel_fb))
        {
            if (params_.yaw_gimbal_joint.enable)
            {
                // Yaw Position (IMU) to velocity (IMU) PID
                yaw_pos_ref = reference_interfaces_[0];
                yaw_pos_err = angles::shortest_angular_distance(yaw_pos_fb, yaw_pos_ref);
                yaw_vel_ref = yaw_pos2vel_pid_->compute_command(yaw_pos_err, period);
                
                command_interfaces_[0].set_value(0.0);
                command_interfaces_[1].set_value(yaw_vel_ref);
                command_interfaces_[2].set_value(0.0);
            }

            if (params_.pitch_gimbal_joint.enable)
            {
                // Pitch Position (IMU) to velocity (IMU) PID
                // for cybergear negative up, for imu(fdilink) negative up, for reference it should be negative up too
                // dm imu is positive up
                pitch_pos_ref = -reference_interfaces_[1];
                pitch_pos_err = angles::shortest_angular_distance(-pitch_pos_fb, pitch_pos_ref);
                command_interfaces_[3].set_value(pitch_enc_pos + pitch_pos_err);
            }
        }
        // Publish state
        if (state_publisher_ && state_publisher_->trylock())
        {
            state_publisher_->msg_.header.stamp = time;
            if (params_.yaw_gimbal_joint.enable)
            {
                state_publisher_->msg_.dof_states[0].reference = yaw_pos_ref;
                state_publisher_->msg_.dof_states[0].feedback = yaw_pos_fb;
                state_publisher_->msg_.dof_states[0].error = yaw_pos_err;
                state_publisher_->msg_.dof_states[0].time_step = period.seconds();
                state_publisher_->msg_.dof_states[0].output = yaw_vel_ref;
            }

            if (params_.pitch_gimbal_joint.enable)
            {
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

PLUGINLIB_EXPORT_CLASS(gimbal_controller::GimbalPositionController, controller_interface::ChainableControllerInterface)
