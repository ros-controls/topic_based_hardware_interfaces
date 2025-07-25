# Joint State Topic Based System

The Joint State Topic Based System implements a ros2_control `hardware_interface::SystemInterface` supporting command and state interfaces through the ROS topic communication layer.

### ros2_control urdf tag

The `joint_state_topic_hardware_interface` has a few `ros2_control` urdf tags to customize its behavior.

#### Parameters

* joint_commands_topic: (default: "/robot_joint_command"). Example: `<param name="joint_commands_topic">/my_topic_joint_commands</param>`.
* joint_states_topic: (default: "/robot_joint_states"). Example: `<param name="joint_states_topic">/my_topic_joint_states</param>`.
* trigger_joint_command_threshold: (default: 1e-5). Used to avoid spamming the joint command topic when the difference between the current joint state and the joint command is smaller than this value, set to -1 to always send the joint command. Example: `<param name="trigger_joint_command_threshold">0.001</param>`.
* sum_wrapped_joint_states: (default: "false"). Used to track the total rotation for joint states the values reported on the `joint_commands_topic` wrap from 2*pi to -2*pi when rotating in the positive direction. (Isaac Sim only reports joint states from 2*pi to -2*pi) Example: `<param name="sum_wrapped_joint_states">true</param>`.

#### Per-joint Parameters

* mimic: Defined name of the joint to mimic. This is often used concept with parallel grippers. Example: `<param name="mimic">joint1</param>`.
* multiplier: Multiplier of values for mimicking joint defined in mimic parameter. Example: `<param name="multiplier">-2</param>`.

### Modifying the urdf `ros2_control` tag for new robots

If your robot description support mock_components you simply add an if-else statement to switch between it and `<plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>`, make sure to add the `joint_commands_topic` and `joint_states_topic` to point to the correct topics.

```xml
        <ros2_control name="name" type="system">
            <hardware>
              <xacro:if value="${mock_hardware}">
                <plugin>mock_components/GenericSystem</plugin>
                <param name="fake_sensor_commands">false</param>
                <param name="state_following_offset">0.0</param>
                <param name="calculate_dynamics">true</param>
              </xacro:if>
              <xacro:unless value="${mock_hardware}">
                <plugin>joint_state_topic_hardware_interface/JointStateTopicSystem</plugin>
                <param name="joint_commands_topic">/topic_based_joint_commands</param>
                <param name="joint_states_topic">/topic_based_joint_states</param>
                <param name="sum_wrapped_joint_states">true</param>
              </xacro:if>
            </hardware>
            <joint name="joint_1">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="joint_n">
                <command_interface name="position"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="mimic_joint_1">
                <param name="mimic">joint_k</param>
                <param name="multiplier">1</param>
                <command_interface name="position" />
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
            <joint name="mimic_joint_n">
                <param name="mimic">joint_kn</param>
                <param name="multiplier">1</param>
                <command_interface name="position" />
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
        </ros2_control>
```
