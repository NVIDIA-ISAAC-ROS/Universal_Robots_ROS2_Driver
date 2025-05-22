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

#ifndef UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
#define UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ur_controllers/impedance_controller_parameters.hpp"

namespace ur_controllers
{
class ImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  void callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  std::shared_ptr<impedance_controller::ParamListener> param_listener_;
  impedance_controller::Params params_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
