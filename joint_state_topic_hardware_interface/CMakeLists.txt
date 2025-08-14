cmake_minimum_required(VERSION 3.16)
project(joint_state_topic_hardware_interface CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
if(WIN32)
  # set the same behavior for windows as it is on linux
  export_windows_symbols()
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  angles
  rclcpp
  hardware_interface
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

# Declare a C++ library
add_library(
  ${PROJECT_NAME}
  src/joint_state_topic_hardware_interface.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
                      angles::angles
                      rclcpp::rclcpp
                      hardware_interface::hardware_interface
                      ${sensor_msgs_TARGETS})
target_include_directories(${PROJECT_NAME}
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  find_package(ros_testing REQUIRED)

  # GTests
  # TODO(anyone): reactivate if https://github.com/ros-controls/topic_based_hardware_interfaces/issues/14 is fixed
  # ament_add_gtest(joint_state_topic_hardware_interface_test test/joint_state_topic_hardware_interface_test.cpp)
  # target_link_libraries(joint_state_topic_hardware_interface_test
  #                       ${PROJECT_NAME}
  #                       angles::angles
  #                       rclcpp::rclcpp
  #                       hardware_interface::hardware_interface
  #                       ${sensor_msgs_TARGETS}
  #                       ros2_control_test_assets::ros2_control_test_assets)

  # Integration tests
  # TODO(anyone): reactivate if https://github.com/ros-controls/topic_based_hardware_interfaces/issues/21 is fixed
  # add_ros_test(
  #   test/ros2_control.test.py
  #   TIMEOUT
  #   120
  #   ARGS
  #   test_file:=${CMAKE_CURRENT_SOURCE_DIR}/test/test_topic_based_robot.py)
endif()

pluginlib_export_plugin_description_file(hardware_interface joint_state_topic_hardware_interface_plugin_description.xml)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
