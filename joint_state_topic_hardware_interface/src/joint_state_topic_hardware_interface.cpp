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
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <angles/angles.h>
#include <joint_state_topic_hardware_interface/joint_state_topic_hardware_interface.hpp>
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

  // Add the corrected delta to the total rotation
  return total_rotation_in + delta;
}
}  // namespace

namespace joint_state_topic_hardware_interface
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
// JointState doesn't contain an acceleration field, so right now it's not used
static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

CallbackReturn JointStateTopicSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Update mimic joints
  // TODO(christophfroehlich): move that to the handles
  const auto& joints = get_hardware_info().joints;
  for (const auto& mimic_joint : get_hardware_info().mimic_joints)
  {
    const auto& mimic_joint_name = joints.at(mimic_joint.joint_index).name;
    const auto& mimicked_joint_name = joints.at(mimic_joint.mimicked_joint_index).name;
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION,
                mimic_joint.offset +
                    mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_POSITION));
    }
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
    }
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION));
    }
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

  topic_based_joint_commands_publisher_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
  topic_based_joint_states_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  // if the values on the `joint_states_topic` are wrapped between -2*pi and 2*pi (like they are in Isaac Sim)
  // sum the total joint rotation returned on the `joint_state_values_` interface
  if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
  {
    sum_wrapped_joint_states_ = true;
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JointStateTopicSystem::read(const rclcpp::Time& /*time*/,
                                                            const rclcpp::Duration& /*period*/)
{
  for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i)
  {
    const auto& joints = get_hardware_info().joints;
    auto it = std::find_if(joints.begin(), joints.end(),
                           [&joint_name = std::as_const(latest_joint_state_.name[i])](
                               const hardware_interface::ComponentInfo& info) { return joint_name == info.name; });
    if (it != joints.end())
    {
      if (std::isfinite(latest_joint_state_.position.at(i)))
      {
        if (sum_wrapped_joint_states_)
        {
          auto name = latest_joint_state_.name[i] + "/" + hardware_interface::HW_IF_POSITION;

          set_state(name, sumRotationFromMinus2PiTo2Pi(latest_joint_state_.position.at(i), get_state(name)));
        }
        else
        {
          set_state(latest_joint_state_.name[i] + "/" + hardware_interface::HW_IF_POSITION,
                    latest_joint_state_.position.at(i));
        }
      }
      if (!latest_joint_state_.velocity.empty() && std::isfinite(latest_joint_state_.velocity.at(i)))
      {
        set_state(latest_joint_state_.name[i] + "/" + hardware_interface::HW_IF_VELOCITY,
                  latest_joint_state_.velocity.at(i));
      }
      if (!latest_joint_state_.effort.empty() && std::isfinite(latest_joint_state_.effort.at(i)))
      {
        set_state(latest_joint_state_.name[i] + "/" + hardware_interface::HW_IF_EFFORT,
                  latest_joint_state_.effort.at(i));
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JointStateTopicSystem::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  // To avoid spamming TopicBased's joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  // const auto diff = std::transform_reduce(
  //     joint_state_values_[POSITION_INTERFACE_INDEX].cbegin(), joint_state_values_[POSITION_INTERFACE_INDEX].cend(),
  //     joint_command_values_[POSITION_INTERFACE_INDEX].cbegin(), 0.0,
  //     [](const auto d1, const auto d2) { return std::abs(d1) + std::abs(d2); }, std::minus<double>{});
  // if (diff <= trigger_joint_command_threshold_)
  // {
  //   return hardware_interface::return_type::OK;
  // }

  sensor_msgs::msg::JointState joint_state;
  const auto& joints = get_hardware_info().joints;
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    joint_state.name.push_back(joints[i].name);
    joint_state.header.stamp = get_node()->now();
    // only send commands to the interfaces that are defined for this joint
    for (const auto& interface : joints[i].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort.push_back(get_command(joints[i].name + "/" + interface.name));
      }
      else
      {
        RCLCPP_WARN_ONCE(get_node()->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         joints[i].name.c_str(), interface.name.c_str());
      }
    }
  }

  // Update mimic joints
  for (const auto& mimic_joint : get_hardware_info().mimic_joints)
  {
    const auto& mimic_joint_name = joints.at(mimic_joint.joint_index).name;
    const auto& mimicked_joint_name = joints.at(mimic_joint.mimicked_joint_index).name;
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_POSITION,
                mimic_joint.offset +
                    mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_POSITION));
    }
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_VELOCITY,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_VELOCITY));
    }
    if (joint_state_interfaces_.find(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION) !=
        joint_state_interfaces_.end())
    {
      set_state(mimic_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION,
                mimic_joint.multiplier * get_state(mimicked_joint_name + "/" + hardware_interface::HW_IF_ACCELERATION));
    }
  }

  if (rclcpp::ok())
  {
    topic_based_joint_commands_publisher_->publish(joint_state);
  }

  return hardware_interface::return_type::OK;
}
}  // end namespace joint_state_topic_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(joint_state_topic_hardware_interface::JointStateTopicSystem, hardware_interface::SystemInterface)
