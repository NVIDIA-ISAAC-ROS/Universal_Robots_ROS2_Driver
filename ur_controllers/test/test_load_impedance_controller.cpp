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

#include <gmock/gmock.h>

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ur_controllers/impedance_controller.hpp"

namespace
{
constexpr size_t JOINT_COUNT = 6;

const std::vector<std::string> JOINTS = { "test_shoulder_pan_joint", "test_shoulder_lift_joint", "test_elbow_joint",
                                          "test_wrist_1_joint",      "test_wrist_2_joint",       "test_wrist_3_joint" };

class ImpedanceControllerBehaviorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    controller_ = std::make_unique<ur_controllers::ImpedanceController>();

    rclcpp::NodeOptions options;
    options.parameter_overrides({ rclcpp::Parameter("tf_prefix", "test_"), rclcpp::Parameter("joints", JOINTS) });
    ASSERT_EQ(controller_->init("test_impedance_controller", "", 100, "", options),
              controller_interface::return_type::OK);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);

    command_values_.assign(JOINT_COUNT, -1.0);
    std::vector<hardware_interface::LoanedCommandInterface> loaned_command_interfaces;
    loaned_command_interfaces.reserve(JOINT_COUNT);
    for (size_t i = 0; i < JOINT_COUNT; ++i) {
      command_interfaces_.emplace_back(std::make_shared<hardware_interface::CommandInterface>(
          "test_impedance", "setpoint_positions_" + std::to_string(i), &command_values_[i]));
      loaned_command_interfaces.emplace_back(command_interfaces_.back(), [] {});
    }
    controller_->assign_interfaces(std::move(loaned_command_interfaces), {});

    ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State()), controller_interface::CallbackReturn::SUCCESS);
    command_publisher_ = controller_->get_node()->create_publisher<sensor_msgs::msg::JointState>(
        "target_joint_positions", rclcpp::SystemDefaultsQoS());
  }

  void TearDown() override
  {
    if (controller_) {
      controller_->on_deactivate(rclcpp_lifecycle::State());
      controller_->release_interfaces();
      controller_.reset();
    }
  }

  void publishAndDispatch(const sensor_msgs::msg::JointState& command)
  {
    for (size_t attempt = 0; attempt < 200 && command_publisher_->get_subscription_count() == 0; ++attempt) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    ASSERT_GT(command_publisher_->get_subscription_count(), 0U);

    command_publisher_->publish(command);
    for (size_t attempt = 0; attempt < 20; ++attempt) {
      rclcpp::spin_some(controller_->get_node()->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  void update()
  {
    ASSERT_EQ(controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.002)),
              controller_interface::return_type::OK);
  }

  std::unique_ptr<ur_controllers::ImpedanceController> controller_;
  std::vector<double> command_values_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
};
}  // namespace

TEST(TestLoadImpedanceController, load_controller)
{
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm{ executor, ros2_control_test_assets::minimal_robot_urdf, true,
                                            "test_controller_manager" };

  const std::string test_file_path = std::string{ TEST_FILES_DIRECTORY } + "/impedance_controller_params.yaml";
  cm.set_parameter({ "test_impedance_controller.params_file", test_file_path });
  cm.set_parameter({ "test_impedance_controller.type", "ur_controllers/ImpedanceController" });

  auto controller = cm.load_controller("test_impedance_controller");
  ASSERT_NE(controller, nullptr);

  const auto command_config = controller->command_interface_configuration();
  EXPECT_EQ(command_config.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_THAT(command_config.names,
              testing::ElementsAre("test_impedance/setpoint_positions_0", "test_impedance/setpoint_positions_1",
                                   "test_impedance/setpoint_positions_2", "test_impedance/setpoint_positions_3",
                                   "test_impedance/setpoint_positions_4", "test_impedance/setpoint_positions_5"));

  const auto state_config = controller->state_interface_configuration();
  EXPECT_EQ(state_config.type, controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(state_config.names.empty());
}

TEST_F(ImpedanceControllerBehaviorTest, ReorderedJointStateWritesConfiguredInterfaceOrder)
{
  sensor_msgs::msg::JointState command;
  command.name = { JOINTS[4], JOINTS[2], JOINTS[0], JOINTS[5], JOINTS[1], JOINTS[3] };
  command.position = { 14.0, 12.0, 10.0, 15.0, 11.0, 13.0 };

  publishAndDispatch(command);
  update();

  EXPECT_THAT(command_values_, testing::ElementsAre(10.0, 11.0, 12.0, 13.0, 14.0, 15.0));
}

TEST_F(ImpedanceControllerBehaviorTest, InvalidJointStateDoesNotPartiallyOverwriteLastValidCommand)
{
  sensor_msgs::msg::JointState valid_command;
  valid_command.name = JOINTS;
  valid_command.position = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 };
  publishAndDispatch(valid_command);
  update();
  ASSERT_THAT(command_values_, testing::ElementsAre(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));

  sensor_msgs::msg::JointState wrong_length;
  wrong_length.name = { JOINTS[0], JOINTS[1], JOINTS[2], JOINTS[3], JOINTS[4] };
  wrong_length.position = { 101.0, 102.0, 103.0, 104.0, 105.0, 106.0 };

  sensor_msgs::msg::JointState unknown_joint;
  unknown_joint.name = { JOINTS[0], JOINTS[1], JOINTS[2], JOINTS[3], JOINTS[4], "unknown_joint" };
  unknown_joint.position = { 101.0, 102.0, 103.0, 104.0, 105.0, 106.0 };

  sensor_msgs::msg::JointState duplicate_joint;
  duplicate_joint.name = { JOINTS[0], JOINTS[1], JOINTS[2], JOINTS[3], JOINTS[4], JOINTS[4] };
  duplicate_joint.position = { 101.0, 102.0, 103.0, 104.0, 105.0, 106.0 };

  sensor_msgs::msg::JointState non_finite_position;
  non_finite_position.name = JOINTS;
  non_finite_position.position = { 101.0, 102.0, 103.0, 104.0, 105.0, std::numeric_limits<double>::quiet_NaN() };

  const std::vector<std::pair<std::string, sensor_msgs::msg::JointState>> invalid_commands = {
    { "wrong length", wrong_length },
    { "unknown joint", unknown_joint },
    { "duplicate joint", duplicate_joint },
    { "non-finite position", non_finite_position },
  };
  for (const auto& [description, command] : invalid_commands) {
    SCOPED_TRACE(description);
    publishAndDispatch(command);
    update();
    EXPECT_THAT(command_values_, testing::ElementsAre(1.0, 2.0, 3.0, 4.0, 5.0, 6.0));
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}
