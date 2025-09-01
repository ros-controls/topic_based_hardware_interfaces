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

import os
from pathlib import Path

import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

SCRIPT_PATH = Path(os.path.realpath(__file__)).parent


def generate_launch_description():
    ros2_controllers_file = Path(SCRIPT_PATH / "ros2_controllers.yaml")
    robot_description = {
        "robot_description": xacro.process_file(SCRIPT_PATH / "rrr.urdf.xacro").toxml(),
    }
    controllers = ["joint_state_broadcaster", "joint_trajectory_controller"]
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[robot_description],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, ros2_controllers_file],
                output="screen",
            ),
        ]
        + [
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
            )
            for controller in controllers
        ],
    )
