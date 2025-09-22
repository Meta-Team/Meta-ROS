#include "meta_utils_controller/armor_tester_controller.hpp"

#include <control_toolbox/pid_ros.hpp>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "behavior_interface/msg/armor.hpp" // unitree_vel, dji_vel
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_VELOCITY;
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();

namespace armor_tester_controller {
    controller_interface::CallbackReturn ArmorTesterController::on_init() {
        // The first line usually calls the parent on_init() method
        // Here is the best place to initialize the variables, reserve memory,
        // and declare node parameters used by the controller

        // call father on_init() method
        // auto ret = ChainableControllerInterface::on_init();
        // if (ret != controller_interface::CallbackReturn::SUCCESS) {
        //     return ret;
        // }

        // declare and set node parameters
        try {
            param_listener_ = std::make_shared<ParamListener> (get_node());
        } catch (const std::exception & e) {
            std::cerr << "Exception thrown during controller's init with message: "
                << e.what() << std::endl;
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn
    ArmorTesterController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
        //  parameters are usually read here
        //  everything is prepared so that the controller can be started.

        // read parameters
        params_ = param_listener_->get_params();

        // initialize dji pid
        RCLCPP_INFO(get_node()->get_logger(), "The params_.dji_joint.name is: %s", params_.dji_joint.name.c_str());
        dji_pid_ = std::make_shared<control_toolbox::PidROS> (
            get_node(), "gains." + params_.dji_joint.name + "_vel2eff", true);
        if (!dji_pid_->initPid()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize PID for DJI Motor");
            return controller_interface::CallbackReturn::FAILURE;
        }

        // topic QoS (Quality of Service)
        auto vel_sub_qos = rclcpp::SystemDefaultsQoS();
        vel_sub_qos.keep_last(1);
        vel_sub_qos.best_effort();
        // create subscriber
        vel_subscriber_ = get_node()->create_subscription<behavior_interface::msg::Armor>(
            "~/reference", vel_sub_qos,
            std::bind(& ArmorTesterController::velocity_callback, this, std::placeholders::_1));
        // create, initialize the message, and write it the real time buffer
        auto vel_msg = std::make_shared<behavior_interface::msg::Armor>();
        vel_msg->dji_vel = NaN;
        vel_msg->unitree_vel = NaN;
        vel_buffer_.writeFromNonRT(vel_msg);

        RCLCPP_INFO(get_node()->get_logger(), "configure successfully");
        return controller_interface::CallbackReturn::SUCCESS;
    }


    controller_interface::InterfaceConfiguration
    ArmorTesterController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration cmd_itf_config;

        cmd_itf_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        cmd_itf_config.names.reserve(2);
        cmd_itf_config.names.push_back(
            params_.unitree_joint.name + "/" + HW_IF_VELOCITY);
        cmd_itf_config.names.push_back(
            params_.dji_joint.name + "/" + HW_IF_EFFORT);

        return cmd_itf_config;
    }

    controller_interface::InterfaceConfiguration
    ArmorTesterController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration state_itf_config;

        state_itf_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        state_itf_config.names.reserve(1);

        // only the dji velocity is required
        state_itf_config.names.push_back(params_.dji_joint.name + "/" + HW_IF_VELOCITY);

        return state_itf_config;
    }

    void ArmorTesterController::velocity_callback(
        const std::shared_ptr<behavior_interface::msg::Armor> msg) {
        // shared_ptr<..>, and still writeFromNonRT(msg)? not writeFromNonRT(*msg)???
        vel_buffer_.writeFromNonRT(msg);
    }

    controller_interface::CallbackReturn
    ArmorTesterController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // set the default value in command
        auto msg_ptr = * (vel_buffer_.readFromRT());
        msg_ptr->unitree_vel = NaN;
        msg_ptr->dji_vel = NaN;

        // initialize the reference_interfaces_, where write the control reference (tgt)
        // in update, the controller implement the control via reference_interfaces_
        reference_interfaces_.assign(reference_interfaces_.size(), NaN);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type
    ArmorTesterController::update_reference_from_subscribers() {
        // shared_ptr must be allocated immediately to avoid dangling
        auto current_vel_msg = * (vel_buffer_.readFromRT());

        if (!std::isnan(current_vel_msg->unitree_vel) &&
            !std::isnan(current_vel_msg->dji_vel)) {
            // set the velocity into the reference_interfaces_
            reference_interfaces_[0] = current_vel_msg->unitree_vel;
            reference_interfaces_[1] = current_vel_msg->dji_vel;

            current_vel_msg->unitree_vel = NaN;
            current_vel_msg->dji_vel = NaN;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn
    ArmorTesterController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        // we use C-c to stop the program,
        // and stop the device in the deconstructor of the hardware interface
        // on_deactivate() here is useless for us.

        return controller_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::CommandInterface>
    ArmorTesterController::on_export_reference_interfaces() {
        reference_interfaces_.resize(2, NaN);

        std::vector<hardware_interface::CommandInterface> ref_itf;
        ref_itf.reserve(reference_interfaces_.size());

        ref_itf.emplace_back(get_node()->get_name(),
                             params_.unitree_joint.name + "/" + HW_IF_VELOCITY,
                             &reference_interfaces_[0]);
        ref_itf.emplace_back(get_node()->get_name(),
                             params_.dji_joint.name + "/" + HW_IF_VELOCITY,
                             &reference_interfaces_[1]);
        return ref_itf;
    }

    controller_interface::return_type
    ArmorTesterController::update_and_write_commands(const rclcpp::Time & /*time*/,
                                                     const rclcpp::Duration & period) {
        if (command_interfaces_.size() != 2 || reference_interfaces_.size() != 2) {
            RCLCPP_WARN(get_node()->get_logger(), "ref_itf or cmd_itf has wrong size");
            // return controller_interface::return_type::ERROR;
        }

        auto ref_unitree_vel = reference_interfaces_[0];
        auto ref_dji_vel = reference_interfaces_[1];
        RCLCPP_INFO(get_node()->get_logger(), "REF:unitree_vel:%.2lf, dji_vel: %.2lf",
                    ref_unitree_vel, ref_dji_vel);
        // write to the command interface for unitree
        if (!std::isnan(ref_unitree_vel)) {
            command_interfaces_[0].set_value(ref_unitree_vel);
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "the reference of unitree velocity is nan");
            // return controller_interface::return_type::ERROR;
        }

        // write to the command interface for dji with pid
        if (!std::isnan(ref_dji_vel)) {
            double current_dji_vel = std::numeric_limits<double>::quiet_NaN();

            for (auto & state_itf : state_interfaces_) {
                // write to the command interface for unitree
                if (state_itf.get_name() == params_.dji_joint.name + "/" + HW_IF_VELOCITY) {
                    current_dji_vel = state_itf.get_value();
                    RCLCPP_INFO(get_node()->get_logger(), "STATE: dji_vel: %lf", current_dji_vel);
                    break;
                }
            }

            if (!std::isnan(current_dji_vel)) {
                double dji_vel_err = ref_dji_vel - current_dji_vel;
                double dji_vel_eff = dji_pid_->computeCommand(dji_vel_err, period);
                RCLCPP_INFO(get_node()->get_logger(), "PID: current_dji_vel: %.2lf, ref_dji_vel: %.2lf, dji_vel_err: %.2lf, dji_vel_eff: %.2lf",
                            current_dji_vel, ref_dji_vel, dji_vel_err, dji_vel_eff);
                command_interfaces_[1].set_value(dji_vel_eff);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "the current dji velocity is nan");
                // return controller_interface::return_type::ERROR;
            }
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "the reference of dji velocity is nan");
            // return controller_interface::return_type::ERROR;
        }

        return controller_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(armor_tester_controller::ArmorTesterController,
                       controller_interface::ChainableControllerInterface)
