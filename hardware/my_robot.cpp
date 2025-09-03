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
  // 조인트 제한값을 저장할 벡터의 크기를 초기화한다.
  hw_joint_limits_.resize(info.joints.size());
  // 각 조인트의 정보를 순회하며 설정 및 유효성 검사를 수행합니다.
  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints[i];

    // Command 인터페이스가 1개인지 확인
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Command 인터페이스가 'position' 타입인지 확인
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // State 인터페이스가 1개인지 확인
    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interfaces. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // State 인터페이스가 'position' 타입인지 확인
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has '%s' state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ▼▼▼ 2. URDF의 <param> 태그에서 min/max 값을 읽어옵니다. ▼▼▼
    try
    {
      hw_joint_limits_[i].min_position = std::stod(joint.parameters.at("min"));
      hw_joint_limits_[i].max_position = std::stod(joint.parameters.at("max"));
    }
    catch (const std::out_of_range &ex)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' has no min/max limits specified in URDF!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument &ex)
    {
      RCLCPP_FATAL(get_logger(), "Joint '%s' min/max limits are not valid numbers!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 읽어온 제한값을 로그로 출력하여 확인
    RCLCPP_INFO(
      get_logger(), "Joint '%s' limits loaded: min=%.3f, max=%.3f",
      joint.name.c_str(), hw_joint_limits_[i].min_position, hw_joint_limits_[i].max_position);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 이 함수는 ros2_control 프레임워크에 이 하드웨어의 어떤 '상태(state)'를 외부에 공개할지 알려주는 역할을 합니다.
// '상태'란 모터 엔코더에서 읽어온 현재 위치, 속도 등 로봇의 실제 값을 의미합니다.
std::vector<hardware_interface::StateInterface> MyRobotSystemHardware::export_state_interfaces()
{
    // 공개할 상태 인터페이스들을 담을 벡터를 생성합니다.
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // URDF에 정의된 모든 조인트에 대해 루프를 실행한다.
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        // 벡터에 새로운 상태 인터페이스를 추가한다.
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]));
    }
    // 첫번째 인자: 조인트 이름 (예: "link1_1_joint")
    // 이 이름을 통ㄴ해 joint_state_broadcaster가 이 조인트의 상태를 구독할 수 있습니다.
    // 두번째 인자: 상태 인터페이스 타입 (예: "position")
    // 세 번째 인자: 실제 데이터가 저장될 변수의 주소.
    // read() 함수가 모터에서 읽어온 위치 값을 pos_[i]에 저장하면,
    // ros2_control 프레임워크는 이 주소를 통해 그 값을 읽어갈 수 있습니다.
    // 설정이 완료된 상태 인터페이스 목록을 반환합니다.

    return state_interfaces;
}


// 이 함수는 ros2_control 프레임워크에 이 하드웨어가 어떤 '명령(command)'을 수신할 수 있는지 알려주는 역할을 합니다.
// '명령'이란 JointTrajectoryController 등 상위 컨트롤러로부터 전달받는 목표 위치, 속도 등을 의미합니다.
std::vector<hardware_interface::CommandInterface> MyRobotSystemHardware::export_command_interfaces()
{
    // 공개할 명령 인터페이스들을 담을 벡터를 생성합니다.
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // URDF에 정의된 모든 조인트에 대해 루프를 실행합니다.
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_[i]));
    }
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

    // 활성화 시 모터가 준비될 시간을 줍니다.
    RCLCPP_INFO(get_logger(), "Waiting for motors to be ready...");
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 모터 원점 설정을 위한 루프
    for (uint8_t i = 1; i < 7; i++) {
      can_driver.write_set_origin(i, false); // 모터 원점 설정 명령
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 500ms 대기
      RCLCPP_INFO(get_logger(), "Motor %d origin set command sent", i);
    }

    if (can_driver.initialize_motor_origin_duty_cycle(1, 0.04f, 0.3f, 10)) {
      RCLCPP_INFO(get_logger(), "Motor Origin initialization Successful");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
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

    // for 루프를 사용하여 모든 조인트에 제한을 적용하고 명령을 전송합니다.
    for (uint i =0; i < 6; ++i)
    {
        // 컨트롤러부터 받은 명령에 조인트 제한을 적용합니다.
        double clamped_command_rad = std::clamp(
          cmd_[i],
          hw_joint_limits_[i].min_position,
          hw_joint_limits_[i].max_position
        );
        // 조인트 제한을 적용한 명령을 모터가 사용하는 단위로 변환한다
        double command_deg = clamped_command_rad * 180.0 / M_PI; // radian to degree 변환

        // 최종 값읋 CAN 드라이버에 전송한다 (모터 ID는 1부터 시작하므로 i+1 사용)
        can_driver.write_position_velocity(i + 1, command_deg, limited_velocity, limited_acceleration);
    }
    
    // 디버그 출력 (필요시)
    /*
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {  // 1초마다 출력
        RCLCPP_INFO(get_logger(), "Velocity: %.1f RPM, Acceleration: %.1f RPM/s", 
                   limited_velocity, limited_acceleration);
    }*/
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