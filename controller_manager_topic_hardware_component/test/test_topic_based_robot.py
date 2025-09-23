#!/usr/bin/env python3

# Copyright 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rclpy
from joint_state_topic_based_robot import JointStateTopicBasedRobot

rclpy.init()

robot = JointStateTopicBasedRobot(["joint_1", "joint_2", "joint_3"])
# By default the joint_states should have the initial_value from rrr.urdf.xacro
current_joint_state = robot.get_current_joint_state()
urdf_initial_values = [0.2, 0.3, 0.1]
assert current_joint_state == urdf_initial_values, (
    f"{current_joint_state=} != {urdf_initial_values=}"
)

# Test setting the robot joint states
joint_state = [0.1, 0.2, 0.3]
robot.set_joint_positions(joint_state)
current_joint_state = robot.get_current_joint_state()
assert current_joint_state == joint_state, f"{current_joint_state=} != {joint_state=}"
