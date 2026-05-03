cmake_minimum_required(VERSION 3.16)
project(braccio_control LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(urdf REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

set_compiler_options()
export_windows_symbols()

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  hardware_interface
)
set(REF_GEN_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
  sensor_msgs
)

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  controller_interface
  realtime_tools
  trajectory_msgs
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_executable(send_trajectory reference_generator/send_trajectory.cpp)

# ament_target_dependencies(send_trajectory rclcpp urdf kdl_parser sensor_msgs)
# target_include_directories(send_trajectory PRIVATE
#   $<TARGET_PROPERTY:urdf,INTERFACE_INCLUDE_DIRECTORIES>
# )

target_link_libraries(send_trajectory PUBLIC
  ${trajectory_msgs_TARGETS}
  kdl_parser::kdl_parser
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  urdf::urdf
)


# zenohcxx
if(CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif()

find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

add_library(
  braccio_control
  SHARED
  hardware/braccio_hardware.cpp
  controller/braccio_controller.cpp
)


target_compile_features(braccio_control PUBLIC cxx_std_17)
target_include_directories(braccio_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
$<INSTALL_INTERFACE:include/braccio_control>
)

target_link_libraries(braccio_control PUBLIC
  ${trajectory_msgs_TARGETS}
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  realtime_tools::realtime_tools
  zenohcxx::zenohc
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface braccio_control.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface braccio_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/braccio_control
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/braccio_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/braccio_control
)
install(
    TARGETS send_trajectory
    RUNTIME DESTINATION lib/braccio_control
)

install(TARGETS braccio_control
  EXPORT export_braccio_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME})

install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  braccio_control/send_trajectory_py.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(braccio_control test/test_urdf_xacro.py)

  # Integration (launch) tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_view_robot_launch.py)
  add_ros_isolated_launch_test(test/test_braccio_controller_launch.py)
endif()

## EXPORTS
ament_export_targets(export_braccio_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS} ${REF_GEN_INCLUDE_DEPENDS} ${CONTROLLER_INCLUDE_DEPENDS})
ament_package()
