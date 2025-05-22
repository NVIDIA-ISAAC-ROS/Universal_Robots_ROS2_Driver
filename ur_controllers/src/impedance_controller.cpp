// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdlib>
#include <string>
#include <unordered_map>

namespace ur_controllers
{

namespace {

static const std::unordered_map<std::string, size_t> JOINTS = {
    {"shoulder_pan_joint", 0},
    {"shoulder_lift_joint", 1},
    {"elbow_joint", 2},
    {"wrist_1_joint", 3},
    {"wrist_2_joint", 4},
    {"wrist_3_joint", 5},
};

}  // namespace

controller_interface::InterfaceConfiguration ImpedanceController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    config.names.emplace_back("shoulder_pan_joint/effort");
    config.names.emplace_back("shoulder_lift_joint/effort");
    config.names.emplace_back("elbow_joint/effort");
    config.names.emplace_back("wrist_1_joint/effort");
    config.names.emplace_back("wrist_2_joint/effort");
    config.names.emplace_back("wrist_3_joint/effort");

    return config;
}

controller_interface::InterfaceConfiguration ImpedanceController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    return config;
}

controller_interface::return_type ImpedanceController::update(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
    return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ImpedanceController::on_init()
{    
  try {
    param_listener_ = std::make_shared<impedance_controller::ParamListener>(this->get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Error encountered during configure");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    const std::string tf_prefix = params_.tf_prefix;

    try
    {
        subscription_ = this->get_node()->create_subscription<sensor_msgs::msg::JointState>(
                tf_prefix + "target_joint_positions", 10,
                std::bind(&ImpedanceController::callback, this, std::placeholders::_1));
    }
    catch (...)
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
    try
    {
        subscription_.reset();
    }
    catch (...)
    {
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

void ImpedanceController::callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (msg->name.size() != command_interfaces_.size() || msg->position.size() != command_interfaces_.size())
    {
        RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid JointState message");
        return;
    }

    for (size_t i = 0; i < msg->name.size(); i++)
    {
        const auto iter = JOINTS.find(msg->name[i]);
        if (iter != JOINTS.end())
        {
            command_interfaces_[iter->second].set_value(msg->position[i]);
        }
        else
        {
            RCLCPP_WARN(this->get_node()->get_logger(),
                        "Skipping unknown joint '%s'", msg->name[i].c_str());
        }
    }
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ImpedanceController, controller_interface::ControllerInterface)
