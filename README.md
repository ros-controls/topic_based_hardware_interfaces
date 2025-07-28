# Topic Based ros2_control Hardware Interfaces
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/topic_based_hardware_interfaces/graph/badge.svg?token=NS73VKPG9V)](https://codecov.io/gh/ros-controls/topic_based_hardware_interfaces)

This is a ROS 2 package for integrating the ros2_control with any system that uses topics to command a robot and publish its state.

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/topic_based_hardware_interfaces/pulls) and review it, or [create your own](https://github.com/ros-controls/topic_based_hardware_interfaces/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started. We are happy to help you with your first contribution.

## Content

- [JointState Topic Hardware Interface](joint_state_topic_hardware_interface/README.md)

## Build status

ROS2 Distro | Branch | Build status | Documentation | Package Build
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`main`](https://github.com/ros-controls/topic_based_hardware_interfaces/tree/main) | [![Rolling Binary Build](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-binary-build.yml/badge.svg?branch=main)](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-binary-build.yml) <br> [![Rolling Semi-Binary Build](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main)](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-semi-binary-build.yml) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__topic_based_hardware_interfaces__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__topic_based_hardware_interfaces__ubuntu_noble_amd64/) | [Documentation](https://control.ros.org/rolling/doc/topic_based_hardware_interfaces/doc/index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary/)
**Kilted** | [`main`](https://github.com/ros-controls/topic_based_hardware_interfaces/tree/main) | see above <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Kdev__topic_based_hardware_interfaces__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Kdev__topic_based_hardware_interfaces__ubuntu_noble_amd64/) | [Documentation](https://control.ros.org/kilted/doc/topic_based_hardware_interfaces/doc/index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary/)
**Jazzy** | [`main`](https://github.com/ros-controls/topic_based_hardware_interfaces/tree/main) | see above <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Jdev__topic_based_hardware_interfaces__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Jdev__topic_based_hardware_interfaces__ubuntu_noble_amd64/) | [Documentation](https://control.ros.org/jazzy/doc/topic_based_hardware_interfaces/doc/index.html) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__topic_based_hardware_interfaces__ubuntu_noble_amd64__binary/)
**Humble** | only supported with building development version of ros2_control | [![Check Rolling Compatibility on Humble](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-compatibility-humble-binary-build.yml/badge.svg)](https://github.com/ros-controls/topic_based_hardware_interfaces/actions/workflows/rolling-compatibility-humble-binary-build.yml) |   |

## Acknowledgements

The project has received major contributions from companies and institutions [listed on control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
