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
import sys
import unittest
from pathlib import Path

import launch_testing
import launch_testing.markers
import pytest
import rclpy
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc

# JointStateTopicBasedRobot is in the same folder as this test
sys.path.insert(0, str(Path(__file__).parent))
from joint_state_topic_based_robot import JointStateTopicBasedRobot


# This function specifies the processes to be run for our test
@pytest.mark.rostest
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            str(Path(os.path.realpath(__file__)).parent),
                            "control.launch.py",
                        ],
                    ),
                ),
            ),
            KeepAliveProc(),
            ReadyToTest(),
        ],
    )


class TestFixture(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.robot = JointStateTopicBasedRobot(["joint_1", "joint_2", "joint_3"])

    def tearDown(self):
        self.robot.destroy_node()

    def test_controller_running(self, proc_output):
        cnames = ["joint_trajectory_controller", "joint_state_broadcaster"]
        check_controllers_running(self.robot, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", ["joint_1", "joint_2", "joint_3"])

    def test_main(self, proc_output):
        # By default the joint_states should have the initial_value from rrr.urdf.xacro
        self.robot.get_logger().info("Checking initial joint states...")
        current_joint_state = self.robot.get_current_joint_state()
        urdf_initial_values = [0.2, 0.3, 0.1]
        assert current_joint_state == urdf_initial_values, (
            f"{current_joint_state=} != {urdf_initial_values=}"
        )

        # Test setting the robot joint states
        self.robot.get_logger().info("Set joint positions...")
        joint_state = [0.1, 0.2, 0.3]
        self.robot.set_joint_positions(joint_state)
        self.robot.get_logger().info("Checking current joint states...")
        current_joint_state = self.robot.get_current_joint_state()
        assert current_joint_state == joint_state, (
            f"{current_joint_state=} != {joint_state=}"
        )


@launch_testing.post_shutdown_test()
class TestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_pass(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
