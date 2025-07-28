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
void sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double& total_rotation)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation, current_wrapped_rad, 2 * M_PI, -2 * M_PI, delta);

  // Add the corrected delta to the total rotation
  total_rotation += delta;
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

  // Initialize storage for all joints' standard interfaces, regardless of their existence and set all values to nan
  joint_commands_.resize(standard_interfaces_.size());
  joint_states_.resize(standard_interfaces_.size());
  for (auto i = 0u; i < standard_interfaces_.size(); i++)
  {
    joint_commands_[i].resize(get_hardware_info().joints.size(), 0.0);
    joint_states_[i].resize(get_hardware_info().joints.size(), 0.0);
  }

  // Initial command values
  for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
  {
    const auto& component = get_hardware_info().joints[i];
    for (const auto& interface : component.state_interfaces)
    {
      auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
      // If interface name is found in the interfaces list
      if (it != standard_interfaces_.end())
      {
        auto index = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
        // Check the initial_value param is used
        if (!interface.initial_value.empty())
        {
          joint_states_[index][i] = std::stod(interface.initial_value);
          joint_commands_[index][i] = std::stod(interface.initial_value);
        }
      }
    }
  }

  // Search for mimic joints
  for (auto i = 0u; i < get_hardware_info().joints.size(); ++i)
  {
    const auto& joint = get_hardware_info().joints.at(i);
    if (joint.parameters.find("mimic") != joint.parameters.cend())
    {
      const auto mimicked_joint_it = std::find_if(
          get_hardware_info().joints.begin(), get_hardware_info().joints.end(),
          [&mimicked_joint = joint.parameters.at("mimic")](const hardware_interface::ComponentInfo& joint_info) {
            return joint_info.name == mimicked_joint;
          });
      if (mimicked_joint_it == get_hardware_info().joints.cend())
      {
        throw std::runtime_error(std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_index = i;
      mimic_joint.mimicked_joint_index =
          static_cast<std::size_t>(std::distance(get_hardware_info().joints.begin(), mimicked_joint_it));
      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        mimic_joint.multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      mimic_joints_.push_back(mimic_joint);
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
  // sum the total joint rotation returned on the `joint_states_` interface
  if (get_hardware_parameter("sum_wrapped_joint_states", "false") == "true")
  {
    sum_wrapped_joint_states_ = true;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JointStateTopicSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
  {
    const auto& joint = get_hardware_info().joints[i];
    for (const auto& interface : joint.state_interfaces)
    {
      // Add interface: if not in the standard list then use "other" interface list
      if (!getInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
      {
        throw std::runtime_error("Interface is not found in the standard list.");
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JointStateTopicSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
  {
    const auto& joint = get_hardware_info().joints[i];
    for (const auto& interface : joint.command_interfaces)
    {
      if (!getInterface(joint.name, interface.name, i, joint_commands_, command_interfaces))
      {
        throw std::runtime_error("Interface is not found in the standard list.");
      }
    }
  }

  return command_interfaces;
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
      auto j = static_cast<std::size_t>(std::distance(joints.begin(), it));
      if (sum_wrapped_joint_states_)
      {
        sumRotationFromMinus2PiTo2Pi(latest_joint_state_.position[i], joint_states_[POSITION_INTERFACE_INDEX][j]);
      }
      else
      {
        joint_states_[POSITION_INTERFACE_INDEX][j] = latest_joint_state_.position[i];
      }
      if (!latest_joint_state_.velocity.empty())
      {
        joint_states_[VELOCITY_INTERFACE_INDEX][j] = latest_joint_state_.velocity[i];
      }
      if (!latest_joint_state_.effort.empty())
      {
        joint_states_[EFFORT_INTERFACE_INDEX][j] = latest_joint_state_.effort[i];
      }
    }
  }

  for (const auto& mimic_joint : mimic_joints_)
  {
    for (auto& joint_state : joint_states_)
    {
      joint_state[mimic_joint.joint_index] = mimic_joint.multiplier * joint_state[mimic_joint.mimicked_joint_index];
    }
  }

  return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool JointStateTopicSystem::getInterface(const std::string& name, const std::string& interface_name,
                                         const size_t vector_index, std::vector<std::vector<double>>& values,
                                         std::vector<HandleType>& interfaces)
{
  auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
  if (it != standard_interfaces_.end())
  {
    auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

hardware_interface::return_type JointStateTopicSystem::write(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  // To avoid spamming TopicBased's joint command topic we check the difference between the joint states and
  // the current joint commands, if it's smaller than a threshold we don't publish it.
  const auto diff = std::transform_reduce(
      joint_states_[POSITION_INTERFACE_INDEX].cbegin(), joint_states_[POSITION_INTERFACE_INDEX].cend(),
      joint_commands_[POSITION_INTERFACE_INDEX].cbegin(), 0.0,
      [](const auto d1, const auto d2) { return std::abs(d1) + std::abs(d2); }, std::minus<double>{});
  if (diff <= trigger_joint_command_threshold_)
  {
    return hardware_interface::return_type::OK;
  }

  sensor_msgs::msg::JointState joint_state;
  for (std::size_t i = 0; i < get_hardware_info().joints.size(); ++i)
  {
    joint_state.name.push_back(get_hardware_info().joints[i].name);
    joint_state.header.stamp = get_node()->now();
    // only send commands to the interfaces that are defined for this joint
    for (const auto& interface : get_hardware_info().joints[i].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position.push_back(joint_commands_[POSITION_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity.push_back(joint_commands_[VELOCITY_INTERFACE_INDEX][i]);
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort.push_back(joint_commands_[EFFORT_INTERFACE_INDEX][i]);
      }
      else
      {
        RCLCPP_WARN_ONCE(get_node()->get_logger(), "Joint '%s' has unsupported command interfaces found: %s.",
                         get_hardware_info().joints[i].name.c_str(), interface.name.c_str());
      }
    }
  }

  for (const auto& mimic_joint : mimic_joints_)
  {
    for (const auto& interface : get_hardware_info().joints[mimic_joint.mimicked_joint_index].command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        joint_state.position[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.position[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        joint_state.velocity[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.velocity[mimic_joint.mimicked_joint_index];
      }
      else if (interface.name == hardware_interface::HW_IF_EFFORT)
      {
        joint_state.effort[mimic_joint.joint_index] =
            mimic_joint.multiplier * joint_state.effort[mimic_joint.mimicked_joint_index];
      }
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
