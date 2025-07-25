// Copyright 2025 ros2_control Development Team
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

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#if __has_include(<hardware_interface/hardware_interface/version.h>)
#include <hardware_interface/hardware_interface/version.h>
#else
#include <hardware_interface/version.h>
#endif
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

TEST(TestTopicBasedSystem, load_topic_based_system_2dof)
{
  const std::string hardware_system_2dof_topic_based =
      R"(
  <ros2_control name="JointStateTopicBasedSystem2dof" type="system">
    <hardware>
      <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
      <param name="joint_commands_topic">/topic_based_joint_commands</param>
      <param name="joint_states_topic">/topic_based_custom_joint_states</param>
    </hardware>
    <joint name="joint1">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2">
      <command_interface name="position"/>
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
)";
  auto urdf =
      ros2_control_test_assets::urdf_head + hardware_system_2dof_topic_based + ros2_control_test_assets::urdf_tail;
  auto node = std::make_shared<rclcpp::Node>("test_topic_based_system");

// The API of the ResourceManager has changed in hardware_interface 5.3.0
#if HARDWARE_INTERFACE_VERSION_GTE(5, 3, 0)
  hardware_interface::ResourceManagerParams params;
  params.robot_description = urdf;
  params.clock = node->get_clock();
  params.logger = node->get_logger();
  params.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(params, true));
// The API of the ResourceManager has changed in hardware_interface 4.13.0
#elif HARDWARE_INTERFACE_VERSION_GTE(4, 13, 0)
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, node->get_node_clock_interface(),
                                                         node->get_node_logging_interface(), false));
#else
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, true, false));
#endif
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
