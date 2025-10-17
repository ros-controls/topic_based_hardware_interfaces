// Copyright 2025 AIT Austrian Institute of Technology GmbH
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

/* Author: Christoph Froehlich */

#include <string>

#include <cm_topic_hardware_component/cm_topic_hardware_component.hpp>

namespace cm_topic_hardware_component
{

CallbackReturn CMTopicSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  // TODO(christophfroehlich): should we use RT box here?
  pal_names_subscriber_ = get_node()->create_subscription<pal_statistics_msgs::msg::StatisticsNames>(
      "~/names", rclcpp::SensorDataQoS(), [this](const pal_statistics_msgs::msg::StatisticsNames::SharedPtr pal_names) {
        pal_statistics_names_per_topic_[pal_names->names_version] = std::move(pal_names->names);
      });
  pal_values_subscriber_ = get_node()->create_subscription<pal_statistics_msgs::msg::StatisticsValues>(
      "~/values", rclcpp::SensorDataQoS(),
      [this](const pal_statistics_msgs::msg::StatisticsValues::SharedPtr pal_values) {
        latest_pal_values_ = *pal_values;
      });

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CMTopicSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  if (latest_pal_values_.names_version == 0 || pal_statistics_names_per_topic_.empty())
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "No data received yet");
    return hardware_interface::return_type::OK;
  }

  auto it = pal_statistics_names_per_topic_.find(latest_pal_values_.names_version);
  auto end_it = pal_statistics_names_per_topic_.end();
  if (it != end_it)
  {
    const auto& names = it->second;
    const size_t N = std::min(names.size(), latest_pal_values_.values.size());
    for (size_t i = 0; i < N; i++)
    {
      // If name begins with "state_interface.", extract the remainder
      const std::string prefix = "state_interface.";
      if (names[i].rfind(prefix, 0) == 0)
      {  // starts with prefix
        const auto& name = names[i].substr(prefix.length());
        if (joint_state_interfaces_.find(name) != joint_state_interfaces_.end() ||
            sensor_state_interfaces_.find(name) != sensor_state_interfaces_.end() ||
            gpio_state_interfaces_.find(name) != gpio_state_interfaces_.end() ||
            unlisted_state_interfaces_.find(name) != unlisted_state_interfaces_.end())
        {
          // TODO(christophfroehlich): does not support other values than double now
          set_state(name, latest_pal_values_.values.at(i));
        }
      }
    }
  }
  else
  {
    RCLCPP_WARN(get_node()->get_logger(), "No matching statistics names found");
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CMTopicSystem::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // nothing to do here
  return hardware_interface::return_type::OK;
}
}  // end namespace cm_topic_hardware_component

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cm_topic_hardware_component::CMTopicSystem, hardware_interface::SystemInterface)
