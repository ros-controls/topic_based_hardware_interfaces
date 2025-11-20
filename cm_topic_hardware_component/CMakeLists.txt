cmake_minimum_required(VERSION 3.16)
project(cm_topic_hardware_component CXX)

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
  pal_statistics_msgs
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
  src/cm_topic_hardware_component.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC
                      angles::angles
                      rclcpp::rclcpp
                      hardware_interface::hardware_interface
                      ${pal_statistics_msgs_TARGETS})
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

  # GTests
  ament_add_gmock(cm_topic_hardware_component_test test/cm_topic_hardware_component_test.cpp)
  target_link_libraries(cm_topic_hardware_component_test
                        ${PROJECT_NAME}
                        rclcpp::rclcpp
                        hardware_interface::hardware_interface
                        ${pal_statistics_msgs_TARGETS}
                        ros2_control_test_assets::ros2_control_test_assets)
endif()

pluginlib_export_plugin_description_file(hardware_interface cm_topic_hardware_component_plugin_description.xml)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
