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

/* Author: Jafar Abdi */

#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

namespace joint_state_topic_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JointStateTopicSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool sum_wrapped_joint_states_{ false };

  /// Use standard interfaces for joints because they are relevant for dynamic behavior
  std::array<std::string, 4> standard_interfaces_ = {
    { hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION,
      hardware_interface::HW_IF_EFFORT }
  };

  struct MimicJoint
  {
    std::size_t joint_index;
    std::size_t mimicked_joint_index;
    double multiplier = 1.0;
  };
  std::vector<MimicJoint> mimic_joints_;

  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<std::vector<double>> joint_commands_;
  std::vector<std::vector<double>> joint_states_;

  // If the difference between the current joint state and joint command is less than this value,
  // the joint command will not be published.
  double trigger_joint_command_threshold_ = 1e-5;

  template <typename HandleType>
  bool getInterface(const std::string& name, const std::string& interface_name, const size_t vector_index,
                    std::vector<std::vector<double>>& values, std::vector<HandleType>& interfaces);
};

}  // namespace joint_state_topic_hardware_interface
