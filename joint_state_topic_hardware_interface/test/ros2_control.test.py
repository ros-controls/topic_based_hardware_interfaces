# -*- coding: utf-8 -*-

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
import unittest
from pathlib import Path

import launch_testing
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

SCRIPT_PATH = Path(os.path.realpath(__file__)).parent


def generate_test_description():
    test_node = Node(
        executable=LaunchConfiguration("test_file"),
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                str(SCRIPT_PATH),
                                "control.launch.py",
                            ],
                        ),
                    ],
                ),
            ),
            DeclareLaunchArgument("test_file"),
            TimerAction(period=2.0, actions=[test_node]),
            launch_testing.util.KeepAliveProc(),
            launch_testing.actions.ReadyToTest(),
        ],
    ), {
        "test_node": test_node,
    }


class TestWaitForCompletion(unittest.TestCase):
    # Waits for test to complete, then waits a bit to make sure result files are generated
    def test_run_complete(self, test_node):
        self.proc_info.assertWaitForShutdown(test_node, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_pass(self, proc_info, test_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)
