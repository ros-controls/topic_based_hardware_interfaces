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
from collections import OrderedDict
from pathlib import Path

import launch_testing
import launch_testing.markers
import numpy as np
import pytest
import rclpy
from ament_index_python.packages import get_package_prefix
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from rclpy.qos import qos_profile_system_default
from sensor_msgs.msg import JointState


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
            Node(
                package="topic_tools",
                executable="relay",
                output="screen",
                arguments=["/topic_based_joint_commands", "/topic_based_joint_states"],
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
        self.node = rclpy.create_node("test_node")

        # Reported joint state from joint_state_broadcaster
        self.current_joint_state_subscriber = self.node.create_subscription(
            JointState,
            "joint_states",
            self.joint_states_callback,
            qos_profile_system_default,
        )

        self.joint_names = ["joint_1", "joint_2", "joint_3"]

    def tearDown(self):
        self.node.destroy_node()

    def test_controller_running(self, proc_output):
        cnames = ["joint_trajectory_controller", "joint_state_broadcaster"]
        check_node_running(self.node, "relay")
        check_controllers_running(self.node, cnames)

    def test_check_if_msgs_published(self):
        check_if_js_published("/joint_states", self.joint_names)

    def test_main(self, launch_service, proc_info, proc_output):
        # By default the joint_states should have the initial_value from rrr.urdf.xacro
        self.node.get_logger().info("Checking initial joint states...")
        current_joint_state = self.get_current_joint_state()
        urdf_initial_values = [0.2, 0.3, 0.1]
        assert current_joint_state == urdf_initial_values, (
            f"{current_joint_state=} != {urdf_initial_values=}"
        )

        # Test setting the robot joint states via controller
        # example_position is run directly here as it was not installed
        pkg_name = "joint_state_topic_hardware_interface"
        proc_action = Node(
            executable=os.path.join(
                get_package_prefix(pkg_name).replace("install", "build"),
                "example_position",
            ),
            output="screen",
        )

        with launch_testing.tools.launch_process(
            launch_service, proc_action, proc_info, proc_output
        ):
            proc_info.assertWaitForShutdown(process=proc_action, timeout=300)
            launch_testing.asserts.assertExitCodes(
                proc_info, process=proc_action, allowable_exit_codes=[0]
            )

        self.node.get_logger().info("Checking final joint states...")
        current_joint_state = self.get_current_joint_state()
        final_values = [0.0, 0.3, 0.1]  # joint_2 and joint_3 should remain unchanged
        assert np.allclose(
            current_joint_state,
            final_values,
            atol=1e-3,
        ), f"{current_joint_state=} != {final_values=}"

    def joint_states_callback(self, msg: JointState):
        self.current_joint_state = self.filter_joint_state_msg(msg)

    def get_current_joint_state(self) -> OrderedDict[str, float]:
        """Get the current joint state reported by ros2_control on joint_states topic."""
        self.current_joint_state = []
        while len(self.current_joint_state) == 0:
            self.node.get_logger().warning(
                f"Waiting for current joint states from topic '{self.current_joint_state_subscriber.topic_name}'...",
                throttle_duration_sec=2.0,
                skip_first=True,
            )
            rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.current_joint_state

    def filter_joint_state_msg(self, msg: JointState):
        joint_states = []
        for joint_name in self.joint_names:
            try:
                index = msg.name.index(joint_name)
            except ValueError:
                msg = f"Joint name '{joint_name}' not in input keys {msg.name}"
                raise ValueError(msg) from None
            joint_states.append(msg.position[index])
        return joint_states


@launch_testing.post_shutdown_test()
class TestProcessPostShutdown(unittest.TestCase):
    # Checks if the test has been completed with acceptable exit codes (successful codes)
    def test_pass(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
