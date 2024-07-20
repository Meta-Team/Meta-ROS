// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Source of this file are templates in
// [RosTeamWorkspace](https://github.com/StoglRobotics/ros_team_workspace) repository.
//

#include "omni_wheel_controller/omni_wheel_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

using ControllerReferenceMsg = omni_wheel_controller::OmniWheelController::ControllerReferenceMsg;

// called from RT control loop
void reset_controller_reference_msg(
  const std::shared_ptr<ControllerReferenceMsg> & msg,
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node)
{
  msg->header.stamp = node->now();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

namespace omni_wheel_controller
{
using hardware_interface::HW_IF_VELOCITY;

OmniWheelController::OmniWheelController() : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn OmniWheelController::on_init()
{
  control_mode_.initRT(control_mode_type::CHASSIS);

  try
  {
    param_listener_ = std::make_shared<omni_wheel_controller::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  omni_wheel_kinematics_ = std::make_unique<OmniWheelKinematics>(
    params_.omni_wheel_angles, params_.omni_wheel_distance, params_.omni_wheel_radius);

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  ref_timeout_ = rclcpp::Duration::from_seconds(params_.reference_timeout);
  ref_subscriber_ = get_node()->create_subscription<ControllerReferenceMsgUnstamped>(
    "~/reference", subscribers_qos,
    std::bind(&OmniWheelController::reference_callback, this, std::placeholders::_1));

  std::shared_ptr<ControllerReferenceMsg> msg = std::make_shared<ControllerReferenceMsg>();
  reset_controller_reference_msg(msg, get_node());
  input_ref_.writeFromNonRT(msg);

  auto set_slow_mode_service_callback =
    [&](
      const std::shared_ptr<ControllerModeSrvType::Request> request,
      std::shared_ptr<ControllerModeSrvType::Response> response)
  {
    if (request->data)
    {
      control_mode_.writeFromNonRT(control_mode_type::CHASSIS);
    }
    else
    {
      control_mode_.writeFromNonRT(control_mode_type::GIMBAL);
    }
    response->success = true;
  };

  set_slow_control_mode_service_ = get_node()->create_service<ControllerModeSrvType>(
    "~/set_slow_control_mode", set_slow_mode_service_callback,
    rmw_qos_profile_services_hist_keep_all);

  try
  {
    // State publisher
    // s_publisher_ =
    //   get_node()->create_publisher<ControllerStateMsg>("~/state", rclcpp::SystemDefaultsQoS());
    // state_publisher_ = std::make_unique<ControllerStatePublisher>(s_publisher_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // TODO(anyone): Reserve memory in state publisher depending on the message type
  // state_publisher_->lock();
  // state_publisher_->msg_.header.frame_id = params_.joints[0];
  // state_publisher_->unlock();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration OmniWheelController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(params_.omni_wheel_joints.size());
  for (const auto & joint : params_.omni_wheel_joints)
  {
    command_interfaces_config.names.push_back(joint + "/" + HW_IF_VELOCITY);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration OmniWheelController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // No feedback required for inverse kinematics

  return state_interfaces_config;
}

void OmniWheelController::reference_callback(const std::shared_ptr<ControllerReferenceMsgUnstamped> msg)
{
  auto stamped_msg = std::make_shared<ControllerReferenceMsg>();
  stamped_msg->header.stamp = get_node()->now();
  stamped_msg->twist = *msg;
  input_ref_.writeFromNonRT(stamped_msg);
}

std::vector<hardware_interface::CommandInterface> OmniWheelController::on_export_reference_interfaces()
{
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

bool OmniWheelController::on_set_chained_mode(bool chained_mode)
{
  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::CallbackReturn OmniWheelController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_reference_msg(*(input_ref_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn OmniWheelController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): depending on number of interfaces, use definitions, e.g., `CMD_MY_ITFS`,
  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type OmniWheelController::update_reference_from_subscribers()
{
  auto current_ref = *(input_ref_.readFromRT()); // A shared_ptr must be allocated immediately to prevent dangling
  const auto command_age = get_node()->now() - current_ref->header.stamp;

  // RCLCPP_INFO(get_node()->get_logger(), "Current reference age: %f", command_age.seconds());

  if (command_age <= ref_timeout_ || ref_timeout_ == rclcpp::Duration::from_seconds(0)) {
    if (!std::isnan(current_ref->twist.linear.x) && 
        !std::isnan(current_ref->twist.linear.y) && 
        !std::isnan(current_ref->twist.angular.z))
    {
      reference_interfaces_[0] = current_ref->twist.linear.x;
      reference_interfaces_[1] = current_ref->twist.linear.y;
      reference_interfaces_[2] = current_ref->twist.angular.z;
    }
  }
  else {
    if (!std::isnan(current_ref->twist.linear.x) && 
        !std::isnan(current_ref->twist.linear.y) && 
        !std::isnan(current_ref->twist.angular.z))
    {
      reference_interfaces_[0] = 0.0;
      reference_interfaces_[1] = 0.0;
      reference_interfaces_[2] = 0.0;
      current_ref->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
      current_ref->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
    }
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type OmniWheelController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{

  if (!std::isnan(reference_interfaces_[0]) &&
      !std::isnan(reference_interfaces_[1]) &&
      !std::isnan(reference_interfaces_[2]))
  {
    auto wheel_vels = omni_wheel_kinematics_->inverse(
      reference_interfaces_[0],
      reference_interfaces_[1],
      reference_interfaces_[2]);
    for (size_t i = 0; i < command_interfaces_.size(); i++) {
      command_interfaces_[i].set_value(wheel_vels[i]);
    }
  }

  reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[1] = std::numeric_limits<double>::quiet_NaN();
  reference_interfaces_[2] = std::numeric_limits<double>::quiet_NaN();

  // if (state_publisher_ && state_publisher_->trylock())
  // {
  //   state_publisher_->msg_.header.stamp = time;
  //   state_publisher_->msg_.set_point = command_interfaces_[CMD_MY_ITFS].get_value();

  //   state_publisher_->unlockAndPublish();
  // }

  return controller_interface::return_type::OK;
}

}  // namespace omni_wheel_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  omni_wheel_controller::OmniWheelController, controller_interface::ChainableControllerInterface)
