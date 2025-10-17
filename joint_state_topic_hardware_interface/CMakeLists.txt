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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(joint_state_topic_hardware_interface_test test/joint_state_topic_hardware_interface_test.cpp)
  target_link_libraries(joint_state_topic_hardware_interface_test
                        ${PROJECT_NAME}
                        angles::angles
                        rclcpp::rclcpp
                        hardware_interface::hardware_interface
                        ${sensor_msgs_TARGETS}
                        ros2_control_test_assets::ros2_control_test_assets)

  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(control_msgs REQUIRED)
  add_executable(jtc_client_position test/rrr/jtc_client_position.cpp)
  target_link_libraries(jtc_client_position PUBLIC
    ${control_msgs_TARGETS}
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
  )
  add_executable(test_gripper test/gripper/test_gripper.cpp)
  target_link_libraries(test_gripper PUBLIC
    ${std_msgs_TARGETS}
    rclcpp::rclcpp
  )
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(
    test/rrr/position.test.py)
  add_ros_isolated_launch_test(
    test/gripper/position.test.py)
endif()

pluginlib_export_plugin_description_file(hardware_interface joint_state_topic_hardware_interface_plugin_description.xml)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
