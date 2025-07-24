cmake_minimum_required(VERSION 3.8)
project(my_robot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 필요한 패키지들을 찾습니다.
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)

# C++ 소스 파일들을 사용하여 공유 라이브러리를 생성합니다.
# 라이브러리 이름은 package.xml의 플러그인 설명과 일치해야 합니다.
add_library(my_robot_ros2_control SHARED
  hardware/my_robot.cpp
  # 여기에 다른 .cpp 파일이 있다면 추가하세요.
)

# 필요한 헤더 파일 경로를 지정합니다.
target_include_directories(my_robot_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 필요한 라이브러리들을 링크합니다.
ament_target_dependencies(my_robot_ros2_control PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
)

# 플러그인 XML 파일을 등록합니다.
pluginlib_export_plugin_description_file(hardware_interface my_robot_ros2_control.xml)

# --- 설치(install) 규칙 ---

# 1. 컴파일된 라이브러리 파일을 lib 폴더에 설치합니다. (이 부분이 누락되었을 겁니다!)
install(
  TARGETS my_robot_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 2. 헤더 파일들을 include 폴더에 설치합니다.
install(
  DIRECTORY hardware/include/
  DESTINATION include/my_robot_ros2_control
)

# 3. 플러그인 XML 파일을 share 폴더에 설치합니다.
install(
  FILES my_robot_ros2_control.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}/bringup
)

install(
  DIRECTORY description
  DESTINATION share/${PROJECT_NAME}
)

# STL 파일들 설치
install(DIRECTORY description/urdf/meshes/
  DESTINATION share/${PROJECT_NAME}/description/urdf/meshes/
)

ament_package()
