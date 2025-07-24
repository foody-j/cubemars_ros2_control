// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 이전에 정의한 RROBT 헤더 파일 포함
#include "include/my_robot_ros2_control/my_robot.hpp"

// CAN 통신 드라이버 헤더 추가
#include "include/my_robot_ros2_control/motor_can_driver.hpp" //CAN 통신 클래스 정의된 헤더
#include "include/my_robot_ros2_control/motor_data.hpp" // 모터 데이터 객체 정의 헤더

// 필요한 표준 라이브러리 헤더들 포함
#include <chrono> // 시간 관련 기능
#include <cmath>  // 수학 함수
#include <iomanip>  // 입출력 포맷팅
#include <limits> //수치 한계 값
#include <memory> // 스마트 포인터
#include <sstream>  // 문자열 스트림
#include <string> // 문자열 처리
#include <vector> // 벡터 컨테이너

// ROS 2 관련 헤더 포함
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_robot_ros2_control
{
  // 하드웨어 초기화 함수 구현 ..
hardware_interface::CallbackReturn MyRobotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (  // 부모 클래스의 초기화가 실패하면 에러 반환
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // 하드웨어 파라미터 설정
  /*
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];  // 왼쪽 바퀴 이름 설정
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  */
  // 일단 이 부분 패스해도 문제 없을 듯. 이미 CAN 에서 초기화 함.
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }
  

  // wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  // wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
std::vector<hardware_interface::StateInterface> MyRobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
  "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &pos_[0]));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //"ak70-10-v1_1_continuous", hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //  wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  //state_interfaces.emplace_back(hardware_interface::StateInterface(
  //wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}
// 위치 명령 인터페이스
std::vector<hardware_interface::CommandInterface> MyRobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "ak70-10-v1_1_continuous", hardware_interface::HW_IF_POSITION, &cmd_[0]));
  //command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}*/

// 멤버 함수로 정확히 정의 - 실제 로봇 조인트 이름에 맞게 수정
std::vector<hardware_interface::StateInterface> MyRobotSystemHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link1_1_joint", hardware_interface::HW_IF_POSITION, &pos_[0]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link2_1_joint", hardware_interface::HW_IF_POSITION, &pos_[1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link3_1_joint", hardware_interface::HW_IF_POSITION, &pos_[2]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link4_1_joint", hardware_interface::HW_IF_POSITION, &pos_[3]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link5_1_joint", hardware_interface::HW_IF_POSITION, &pos_[4]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "link6_1_joint", hardware_interface::HW_IF_POSITION, &pos_[5]));
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotSystemHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link1_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[0]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link2_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link3_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[2]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link4_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[3]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link5_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[4]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "link6_1_joint", hardware_interface::HW_IF_POSITION, &cmd_[5]));
    return command_interfaces;
}

hardware_interface::CallbackReturn MyRobotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (can_driver.connected())
  {
    can_driver.disconnect();
  }
  can_driver.connect();
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  if (can_driver.connected())
  {
    can_driver.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MyRobotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/) {
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");
    // 여기에 속도와 가속도 초기화 추가
    velocity_ = 5.0f;  // RPM
    acceleration_ = 10.0f;  // RPM/s    

    if (!can_driver.connected()) {
        can_driver.connect();
    }
    
    // 원점 설정 전에 연결 확인
    if (!can_driver.connected()) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to CAN bus");
        return hardware_interface::CallbackReturn::ERROR;
    }
    /*
    if (can_driver.initialize_motor_origin(1)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(1, 0.0, velocity_, acceleration_);
    }

    if (can_driver.initialize_motor_origin(2)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(2, 0.0, velocity_, acceleration_);
    }

    if (can_driver.initialize_motor_origin(3)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(3, 0.0, velocity_, acceleration_);
    }

    if (can_driver.initialize_motor_origin(4)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(4, 0.0, velocity_, acceleration_);
    }

    if (can_driver.initialize_motor_origin(5)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(5, 0.0, velocity_, acceleration_);
    }
    if (can_driver.initialize_motor_origin(6)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      can_driver.write_position_velocity(6, 0.0, velocity_, acceleration_);
    }
    else {
        RCLCPP_ERROR(get_logger(), "Failed to initialize motor origin");
        return hardware_interface::CallbackReturn::ERROR;
    }*/
    
    RCLCPP_INFO(get_logger(), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MyRobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  //comms_.disconnect(); // 연결 끊기
  can_driver.disconnect();
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MyRobotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    if (!can_driver.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  // <<< 핵심 수정 부분 >>>
  // 각 모터의 최신 상태를 읽어서 pos_ 배열을 업데이트
  for (uint8_t i = 1; i < 7; i++)  
  {
      motor_data = can_driver.getMotorData(i);
      // pos_[i-1] = motor_data.position;
      pos_[i-1] = motor_data.position * M_PI / 180.0;  // degree를 radian으로 변환
      spd_[i-1] = motor_data.speed;
      
      std::cout << std::dec;  // 10진수 모드로 명시적 설정
      std::cout << "Motor " << static_cast<int>(motor_data.motor_id) << ": "
          << "Position: " << pos_[i-1] << " 라디안 "
          << "Speed: " << spd_[i-1] << " RPM "
          << "Current: " << motor_data.current << "A "
          << "Temperature: " << static_cast<int>(motor_data.temperature) << "°C "
          << "Error: 0x" << std::hex << static_cast<int>(motor_data.error) 
          << std::dec << std::endl;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!can_driver.connected()) {
    return hardware_interface::return_type::ERROR;
  }
  
  try {
    // ⭐ 동적으로 속도 제한 (급격한 변화 방지)
    const float MAX_VELOCITY = 3.0f;    // 최대 3 RPM으로 제한
    const float MAX_ACCELERATION = 5.0f; // 최대 5 RPM/s로 제한
    
    float limited_velocity = std::min(velocity_, MAX_VELOCITY);
    float limited_acceleration = std::min(acceleration_, MAX_ACCELERATION);
    
    // radian to degree 변환
    double degree1 = cmd_[0] * 180.0 / M_PI;
    can_driver.write_position_velocity(1, degree1, limited_velocity, limited_acceleration);

    double degree2 = cmd_[1] * 180.0 / M_PI;
    can_driver.write_position_velocity(2, degree2, limited_velocity, limited_acceleration);

    double degree3 = cmd_[2] * 180.0 / M_PI;
    can_driver.write_position_velocity(3, degree3, limited_velocity, limited_acceleration);

    double degree4 = cmd_[3] * 180.0 / M_PI;
    can_driver.write_position_velocity(4, degree4, limited_velocity, limited_acceleration);

    double degree5 = cmd_[4] * 180.0 / M_PI;
    can_driver.write_position_velocity(5, degree5, limited_velocity, limited_acceleration);

    double degree6 = cmd_[5] * 180.0 / M_PI;
    can_driver.write_position_velocity(6, degree6, limited_velocity, limited_acceleration);
    
    // 디버그 출력 (필요시)
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {  // 1초마다 출력
        RCLCPP_INFO(get_logger(), "Velocity: %.1f RPM, Acceleration: %.1f RPM/s", 
                   limited_velocity, limited_acceleration);
    }
  }
  catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to write command: %s", e.what());
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace my_robot_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_robot_ros2_control::MyRobotSystemHardware, hardware_interface::SystemInterface)