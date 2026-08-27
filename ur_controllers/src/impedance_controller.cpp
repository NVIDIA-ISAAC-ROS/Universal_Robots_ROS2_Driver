// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "ur_controllers/impedance_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iterator>
#include <string>

namespace ur_controllers
{
controller_interface::CallbackReturn ImpedanceController::on_init()
{
  try {
    param_listener_ = std::make_shared<impedance_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
    if (params_.joints.size() != JOINT_COUNT) {
      RCLCPP_ERROR(get_node()->get_logger(), "The impedance controller requires exactly six configured joints.");
      return CallbackReturn::ERROR;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    config.names.emplace_back(params_.tf_prefix + "impedance/setpoint_positions_" + std::to_string(i));
  }

  return config;
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn
ImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return CallbackReturn::ERROR;
  }

  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  command_subscriber_ = get_node()->create_subscription<JointState>(
      "target_joint_positions", rclcpp::SystemDefaultsQoS(),
      std::bind(&ImpedanceController::command_callback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ImpedanceController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  command_received_ = false;
  subscriber_is_active_ = true;
  return ControllerInterface::on_activate(previous_state);
}

controller_interface::CallbackReturn
ImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  subscriber_is_active_ = false;
  command_received_ = false;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ImpedanceController::update(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  if (!command_received_.load()) {
    return controller_interface::return_type::OK;
  }

  const auto command = received_command_.try_get();
  if (!command.has_value()) {
    return controller_interface::return_type::OK;
  }

  bool success = true;
  for (size_t i = 0; i < JOINT_COUNT; ++i) {
    success &= command_interfaces_[i].set_value(command.value()[i]);
  }

  if (!success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to write to impedance command interfaces.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void ImpedanceController::command_callback(const JointState::SharedPtr msg)
{
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new impedance commands. Subscriber is inactive.");
    return;
  }

  if (msg->name.size() != JOINT_COUNT || msg->position.size() != JOINT_COUNT) {
    RCLCPP_ERROR(get_node()->get_logger(), "Impedance commands require exactly six joint names and positions.");
    return;
  }

  Command command{};
  std::array<bool, JOINT_COUNT> assigned{};
  for (size_t message_index = 0; message_index < JOINT_COUNT; ++message_index) {
    const auto joint = std::find(params_.joints.begin(), params_.joints.end(), msg->name[message_index]);
    if (joint == params_.joints.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Unknown joint '%s' in impedance command.",
                   msg->name[message_index].c_str());
      return;
    }

    const auto joint_index = static_cast<size_t>(std::distance(params_.joints.begin(), joint));
    if (assigned[joint_index]) {
      RCLCPP_ERROR(get_node()->get_logger(), "Duplicate joint '%s' in impedance command.",
                   msg->name[message_index].c_str());
      return;
    }
    if (!std::isfinite(msg->position[message_index])) {
      RCLCPP_ERROR(get_node()->get_logger(), "Non-finite position for joint '%s' in impedance command.",
                   msg->name[message_index].c_str());
      return;
    }

    assigned[joint_index] = true;
    command[joint_index] = msg->position[message_index];
  }

  received_command_.set(command);
  command_received_ = true;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ImpedanceController, controller_interface::ControllerInterface)
