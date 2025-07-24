cmake_minimum_required(VERSION 3.8)
project(my_robot_ros2_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
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
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  my_robot_ros2_control
  SHARED
  hardware/my_robot.cpp
)
target_compile_features(my_robot_ros2_control PUBLIC cxx_std_17)
target_include_directories(my_robot_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/my_robot_ros2_control>
)
ament_target_dependencies(
  my_robot_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface my_robot_ros2_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/my_robot_ros2_control
)

install(
  DIRECTORY description
  DESTINATION share/my_robot_ros2_control
)

# STL 파일들 설치
install(DIRECTORY description/urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/description/urdf/meshes/
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/my_robot_ros2_control
)

install(
  DIRECTORY rviz
  DESTINATION share/my_robot_ros2_control
)
