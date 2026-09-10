// Copyright 2026 ros2_control Development Team
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

/* Author: Kamalkant Thangaraju */
#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <joint_command_topic_hardware_interface/joint_command_topic_hardware_interface.hpp>
#include <rclcpp/executors.hpp>

namespace
{
/** @brief Sums the total rotation for joint states that wrap from 2*pi to -2*pi
when rotating in the positive direction */
double sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double total_rotation_in)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation_in, current_wrapped_rad, 2 * M_PI, -2 * M_PI,
                                                      delta);

  return total_rotation_in + delta;
}
}  // namespace

namespace joint_command_topic_hardware_interface
{

CallbackReturn JointCommandTopicSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = get_hardware_info().hardware_parameters.find(parameter_name);
        it != get_hardware_info().hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  };

  if (auto it = get_hardware_info().hardware_parameters.find("trigger_joint_command_threshold");
      it != get_hardware_info().hardware_parameters.end())
  {
    trigger_joint_command_threshold_ = std::stod(it->second);
  }

  topic_based_joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) {
        latest_joint_state_.writeFromNonRT(*joint_state);
      });

  const auto joint_commands_topic = get_hardware_parameter("joint_commands_topic", "/robot_joint_commands");
  const auto& joints = get_hardware_info().joints;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    for (const auto& interface : joints[i].command_interfaces)
    {
      const bool supported_command_interface = interface.name == hardware_interface::HW_IF_POSITION ||
                                               interface.name == hardware_interface::HW_IF_VELOCITY ||
                                               interface.name == hardware_interface::HW_IF_EFFORT;
      if (!supported_command_interface)
      {
        RCLCPP_WARN_ONCE(get_node()->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         joints[i].name.c_str(), interface.name.c_str());
        continue;
      }
      if (topic_based_joint_command_publishers_.find(interface.name) == topic_based_joint_command_publishers_.end())
      {
        topic_based_joint_command_publishers_[interface.name] =
            get_node()->create_publisher<control_msgs::msg::JointCommand>(joint_commands_topic + "/" + interface.name,
                                                                          rclcpp::QoS(1));
      }
      auto& group = command_groups_[interface.name];
      group.interface_name = interface.name;
      group.msg.interface_name = interface.name;
      group.joint_names.push_back(joints[i].name);
      group.command_keys.push_back(joints[i].name + "/" + interface.name);
    }
  }

  // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
  if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
  {
    sum_wrapped_joint_states_ = true;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JointCommandTopicSystem::read(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  const auto& joints = get_hardware_info().joints;
  const auto& joint_state = *latest_joint_state_.readFromRT();
  for (std::size_t i = 0; i < joint_state.name.size(); ++i)
  {
    const auto it = std::find_if(joints.begin(), joints.end(),
                                 [name = joint_state.name[i]](const hardware_interface::ComponentInfo& joint) {
                                   return joint.name == name;
                                 });
    if (it != joints.end())
    {
      if (std::find_if(get_hardware_info().mimic_joints.begin(), get_hardware_info().mimic_joints.end(),
                       [idx = static_cast<std::size_t>(std::distance(joints.begin(), it))](
                           const hardware_interface::MimicJoint& mimic_joint) {
                         return idx == mimic_joint.joint_index;
                       }) != get_hardware_info().mimic_joints.end())
      {
        // mimic joints are updated at the end of this function
        continue;
      }

      // these arrays are allowed to be shorter than name
      if (i < joint_state.position.size() && std::isfinite(joint_state.position[i]))
      {
        if (sum_wrapped_joint_states_)
        {
          auto name = joint_state.name[i] + "/" + hardware_interface::HW_IF_POSITION;

          set_state(name, sumRotationFromMinus2PiTo2Pi(joint_state.position[i], get_state(name)));
        }
        else
        {
          set_state(joint_state.name[i] + "/" + hardware_interface::HW_IF_POSITION, joint_state.position[i]);
        }
      }
      if (i < joint_state.velocity.size() && std::isfinite(joint_state.velocity[i]))
      {
        set_state(joint_state.name[i] + "/" + hardware_interface::HW_IF_VELOCITY, joint_state.velocity[i]);
      }
      if (i < joint_state.effort.size() && std::isfinite(joint_state.effort[i]))
      {
        set_state(joint_state.name[i] + "/" + hardware_interface::HW_IF_EFFORT, joint_state.effort[i]);
      }
    }
  }

  for (const auto& mimic_joint : get_hardware_info().mimic_joints)
  {
    const auto& mimic_joint_name = joints.at(mimic_joint.joint_index).name;
    const auto& mimicked_joint_name = joints.at(mimic_joint.mimicked_joint_index).name;
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION))
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION,
                mimic_joint.offset +
                    mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_POSITION));
    }
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY))
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
    }
    if (has_state(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION))
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION));
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JointCommandTopicSystem::write(const rclcpp::Time& /*time*/,
                                                               const rclcpp::Duration& /*period*/)
{
  const auto& joints = get_hardware_info().joints;
  // To avoid spamming the joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  auto diff = 0.0;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    for (const auto& interface : joints[i].command_interfaces)
    {
      const bool supported_command_interface = interface.name == hardware_interface::HW_IF_POSITION ||
                                               interface.name == hardware_interface::HW_IF_VELOCITY ||
                                               interface.name == hardware_interface::HW_IF_EFFORT;
      if (!supported_command_interface)
      {
        continue;
      }
      const auto interface_key = joints[i].name + "/" + interface.name;
      // nothing to compare against
      if (!has_state(interface_key))
      {
        continue;
      }
      // undriven interfaces are NaN and would make diff NaN too
      const auto command = get_command(interface_key);
      if (!std::isfinite(command))
      {
        continue;
      }
      diff += std::abs(get_state(interface_key) - command);
    }
  }
  if (diff <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  if (rclcpp::ok())
  {
    for (auto& [interface_name, group] : command_groups_)
    {
      auto& msg = group.msg;
      msg.joint_names.clear();
      msg.values.clear();
      for (std::size_t i = 0; i < group.command_keys.size(); ++i)
      {
        const auto command = get_command(group.command_keys[i]);
        if (!std::isfinite(command))
        {
          continue;
        }
        msg.joint_names.push_back(group.joint_names[i]);
        msg.values.push_back(command);
      }
      if (msg.joint_names.empty())
      {
        continue;
      }
      msg.header.stamp = get_node()->now();
      topic_based_joint_command_publishers_.at(interface_name)->publish(msg);
    }
  }

  return hardware_interface::return_type::OK;
}
}  // end namespace joint_command_topic_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(joint_command_topic_hardware_interface::JointCommandTopicSystem,
                       hardware_interface::SystemInterface)
