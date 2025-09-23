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
#include <rclcpp/subscription.hpp>

#include <pal_statistics_msgs/msg/statistics_names.hpp>
#include <pal_statistics_msgs/msg/statistics_values.hpp>

namespace controller_manager_topic_hardware_component
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CMTopicSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) override;

private:
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsNames>::SharedPtr pal_names_subscriber_;
  rclcpp::Subscription<pal_statistics_msgs::msg::StatisticsValues>::SharedPtr pal_values_subscriber_;
  pal_statistics_msgs::msg::StatisticsValues latest_pal_values_;
  std::unordered_map<uint32_t, std::vector<std::string>> pal_statistics_names_per_topic_;
};

}  // namespace controller_manager_topic_hardware_component
