# Joint Command Topic Based System

The Joint Command Topic Based System implements a ros2_control `hardware_interface::SystemInterface` supporting command and state interfaces through the ROS topic communication layer. It publishes commands as `control_msgs/JointCommand` messages, one topic per interface type. Only `position`, `velocity` and `effort` command interfaces are supported, anything else is ignored with a warning.

## ros2_control urdf tag

The `joint_command_topic_hardware_interface` has a few `ros2_control` urdf tags to customize its behavior.

### Parameters

* joint_commands_topic: (default: "/robot_joint_commands"). Base topic for the joint command topics. Example: `<param name="joint_commands_topic">/my_topic_joint_commands</param>`.
* joint_states_topic: (default: "/robot_joint_states"). Example: `<param name="joint_states_topic">/my_topic_joint_states</param>`.
* trigger_joint_command_threshold: (default: 1e-5). Used to avoid spamming the joint command topic when the difference between the current joint state and the joint command is smaller than this value, set to -1 to always send the joint command. Example: `<param name="trigger_joint_command_threshold">0.001</param>`.
* sum_wrapped_joint_states: (default: "false"). Used to track the total rotation when the position values reported on the `joint_states_topic` wrap from 2*pi to -2*pi when rotating in the positive direction. (Isaac Sim only reports joint states from 2*pi to -2*pi) Only `position` states are affected. Example: `<param name="sum_wrapped_joint_states">true</param>`.

### Mimic joints

Often used with parallel grippers. The relation is declared in the URDF, not with `ros2_control` params:

```xml
<joint name="joint2" type="revolute">
    <mimic joint="joint1" multiplier="-2" offset="0"/>
    ...
</joint>
```

Then mark the joint with `mimic="true"` and give it state interfaces only. Its states come from the mimicked joint instead of the `joint_states_topic`:

```xml
<joint name="joint2" mimic="true">
    <state_interface name="position"/>
    <state_interface name="velocity"/>
</joint>
```

`offset` only applies to `position`. `velocity` and `acceleration` use `multiplier` alone.

## Example

```xml
        <ros2_control name="name" type="system">
            <hardware>
              <plugin>joint_command_topic_hardware_interface/JointCommandTopicSystem</plugin>
              <param name="joint_commands_topic">/topic_based_joint_commands</param>
              <param name="joint_states_topic">/topic_based_joint_states</param>
            </hardware>
            <joint name="joint_1">
                <command_interface name="position"/>
                <command_interface name="velocity"/>
                <state_interface name="position">
                  <param name="initial_value">0.0</param>
                </state_interface>
                <state_interface name="velocity"/>
            </joint>
            ...
        </ros2_control>
```

## Topics

Each `write()` publishes one `control_msgs/JointCommand` per interface type, on topics derived from `joint_commands_topic`:

* `<joint_commands_topic>/position`: joints with a driven `position` command.
* `<joint_commands_topic>/velocity`: joints with a driven `velocity` command.
* `<joint_commands_topic>/effort`: joints with a driven `effort` command.

`joint_names` and `values` line up, `interface_name` repeats the interface. With the defaults, a joint with `position` and `velocity` commands publishes to `/robot_joint_commands/position` and `/robot_joint_commands/velocity`.

Command interfaces that no controller writes to hold `NaN`. Those joints are dropped from the message, and if nothing drives an interface type at all its topic stays quiet. Declaring a command interface is not enough to get traffic on the matching topic.

Nothing is published while the summed state-command difference stays at or below `trigger_joint_command_threshold`, which is the normal case once a controller has converged. Set it to -1 to publish every cycle.

### QoS

Command publishers are reliable with a depth of 1, so don't count on them to buffer.

The `joint_states_topic` subscription uses `SensorDataQoS` (best effort, depth 5), which any joint state publisher can match.
