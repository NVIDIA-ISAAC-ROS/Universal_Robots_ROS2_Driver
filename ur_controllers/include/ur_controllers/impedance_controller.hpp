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

#ifndef UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
#define UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <memory>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "ur_controllers/impedance_controller_parameters.hpp"

namespace ur_controllers
{
class ImpedanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

private:
  static constexpr size_t JOINT_COUNT = 6;
  using Command = std::array<double, JOINT_COUNT>;
  using JointState = sensor_msgs::msg::JointState;

  void command_callback(const JointState::SharedPtr msg);

  std::shared_ptr<impedance_controller::ParamListener> param_listener_;
  impedance_controller::Params params_;

  rclcpp::Subscription<JointState>::SharedPtr command_subscriber_;
  realtime_tools::RealtimeThreadSafeBox<Command> received_command_;
  std::atomic<bool> subscriber_is_active_{ false };
  std::atomic<bool> command_received_{ false };
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__IMPEDANCE_CONTROLLER_HPP_
