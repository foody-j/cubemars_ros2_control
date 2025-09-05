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
      std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100ms 대기
      RCLCPP_INFO(get_logger(), "Motor %d origin set command sent", i);
    }

    // === 순차적 모터 원점 초기화 ===
    RCLCPP_INFO(get_logger(), "모터 원점 초기화를 시작합니다...");
    
    // 초기화할 모터 수 정의 (현재는 1개만 테스트)
    const uint8_t MOTOR_COUNT = 6;
    const uint8_t START_MOTOR_ID = 1;
    const int MAX_RETRY_COUNT = 2; // 최대 재시도 횟수

    // 각 모터별 초기화 매개변수 (필요에 따라 모터별로 다르게 설정 가능)
    struct MotorInitParams {
        float duty_cycle;
        float speed_threshold;
        int timeout_seconds;
    };

    // 모터별 초기화 파라미터 배열 (현재는 모터 1개만)
    MotorInitParams motor_params[MOTOR_COUNT] = {
      {0.04f, 0.3f, 10},  // 모터 1
      // 나중에 모터 수를 늘릴 때 아래 항목들을 추가하고 MOTOR_COUNT도 변경
      {-0.02f, 0.2f, 10},  // 모터 2 
      {0.02f, 0.3f, 10},  // 모터 3
      {0.04f, 0.3f, 10},  // 모터 4
      {0.03f, 0.3f, 10},  // 모터 5
      {0.0f, 0.3f, 10}   // 모터 6
    }; // 세미콜론 추가!
    
    // === 다른 모든 모터를 정지시키는 헬퍼 함수 ===
    auto stop_all_other_motors = [&](uint8_t current_motor_id) {
        // 전체 모터 범위에서 정지 (1-6번)
        for (uint8_t i = 1; i <= 6; i++) {
            if (i != current_motor_id) {
                try {
                    // 스피드 루프로 0을 줘서 완전 정지
                    can_driver.write_velocity(i, 0.0f);
                    RCLCPP_DEBUG(get_logger(), "모터 %d 안전 정지 완료", i);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(get_logger(), "모터 %d 정지 중 오류: %s", i, e.what());
                }
            }
        }
        // 정지 명령이 반영될 시간 제공
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    };
    
    // === 각 모터를 순차적으로 초기화 (재시도 포함) ===
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        uint8_t motor_id = START_MOTOR_ID + i;
        bool motor_success = false;
        
        RCLCPP_INFO(get_logger(), "=== 모터 %d 원점 초기화 시작 ===", motor_id);
        
        // 현재 진행 중인 모터를 제외한 모든 모터 정지
        RCLCPP_INFO(get_logger(), "다른 모든 모터를 안전 정지시킵니다...");
        stop_all_other_motors(motor_id);
        
        // 재시도 루프 (최대 MAX_RETRY_COUNT번)
        for (int retry = 0; retry < MAX_RETRY_COUNT && !motor_success; retry++) {
            if (retry > 0) {
                RCLCPP_WARN(get_logger(), "모터 %d 원점 초기화 재시도 %d/%d", 
                           motor_id, retry + 1, MAX_RETRY_COUNT);
                
                // 재시도 전 추가 안정화 시간
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            
            try {
                // 현재 모터의 매개변수 가져오기
                const auto& params = motor_params[i];
                
                RCLCPP_INFO(get_logger(), "모터 %d 원점 초기화 시도 중... (시도 %d/%d)", 
                           motor_id, retry + 1, MAX_RETRY_COUNT);
                
                // 원점 초기화 함수 호출
                bool success = can_driver.initialize_motor_origin_duty_cycle(
                    motor_id, 
                    params.duty_cycle, 
                    params.speed_threshold, 
                    params.timeout_seconds
                );
                
                if (success) {
                    motor_success = true;
                    RCLCPP_INFO(get_logger(), "✅ 모터 %d 원점 초기화 성공! (시도 %d/%d)", 
                               motor_id, retry + 1, MAX_RETRY_COUNT);
                    
                    // 성공 후 모터 완전 정지 확인
                    can_driver.write_velocity(motor_id, 0.0f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(300));
                    
                } else {
                    RCLCPP_ERROR(get_logger(), "❌ 모터 %d 원점 초기화 실패 (시도 %d/%d)", 
                                motor_id, retry + 1, MAX_RETRY_COUNT);
                }
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "모터 %d 원점 초기화 중 예외 발생 (시도 %d/%d): %s", 
                            motor_id, retry + 1, MAX_RETRY_COUNT, e.what());
            }
        }
        
        // === 모터별 최종 결과 처리 ===
        if (!motor_success) {
            RCLCPP_ERROR(get_logger(), "💥 모터 %d 원점 초기화 최종 실패! (최대 재시도 %d회 모두 실패)", 
                        motor_id, MAX_RETRY_COUNT);
            RCLCPP_ERROR(get_logger(), "시스템 안전을 위해 활성화를 중단합니다.");
            
            // 실패 시 모든 모터 즉시 정지 (전체 범위 1-6)
            for (uint8_t j = 1; j <= 6; j++) {
                try {
                    can_driver.write_velocity(j, 0.0f);
                } catch (...) {
                    // 정지 명령 실패해도 계속 진행 (안전상 중요)
                }
            }
            
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(get_logger(), "모터 %d 완료. 다음 모터로 진행합니다...", motor_id);
        
        // 다음 모터 초기화 전 시스템 안정화
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // === 모든 모터 초기화 완료 ===
    RCLCPP_INFO(get_logger(), "🎉 모든 모터 원점 초기화 완료!");
    
    // 최종 안전 확인: 모든 모터 정지 (전체 범위 1-6)
    RCLCPP_INFO(get_logger(), "최종 안전 점검: 모든 모터 정지 확인...");
    for (uint8_t i = 1; i <= 6; i++) {
        try {
            can_driver.write_velocity(i, 0.0f);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "모터 %d 최종 정지 확인 중 오류: %s", i, e.what());
        }
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