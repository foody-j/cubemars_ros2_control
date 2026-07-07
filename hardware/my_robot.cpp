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
#include <algorithm>
#include <chrono> // 시간 관련 기능
#include <cctype>
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
namespace
{
void command_j2j4_free_duty(CanComms & can_driver)
{
    can_driver.write_duty_cycle(2, 0.0f);
    can_driver.write_duty_cycle(4, 0.0f);
}

void command_all_free_duty(CanComms & can_driver)
{
    for (uint8_t i = 1; i <= 6; ++i) {
        can_driver.write_duty_cycle(i, 0.0f);
    }
}

void command_non_j2j4_hold(CanComms & can_driver)
{
    for (uint8_t i : {1, 3, 5, 6}) {
        can_driver.write_velocity(i, 0.0f);
    }
}
}  // namespace

// System state management functions
void MyRobotSystemHardware::set_system_state(SystemState new_state)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (system_state_ != new_state) {
        auto old_state = system_state_;
        system_state_ = new_state;
        RCLCPP_INFO(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "🔄 System state: %s → %s",
            system_state_to_string(old_state).c_str(),
            system_state_to_string(new_state).c_str());
    }
}

SystemState MyRobotSystemHardware::get_system_state() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return system_state_;
}

std::string MyRobotSystemHardware::system_state_to_string(SystemState state) const
{
    switch (state) {
        case SystemState::IDLE:    return "IDLE";
        case SystemState::HOMING:  return "HOMING";
        case SystemState::READY:   return "READY";
        case SystemState::RUNNING: return "RUNNING";
        case SystemState::ERROR:   return "ERROR";
        default:                   return "UNKNOWN";
    }
}

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

    bool has_position_command = false;
    bool has_velocity_command = false;
    for (const auto & interface : joint.command_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_command = true;
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_command = true;
      } else {
        RCLCPP_FATAL(
          rclcpp::get_logger("MyRobotSystemHardware"),
          "Joint '%s' has unsupported command interface '%s'.",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!has_position_command || !has_velocity_command)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyRobotSystemHardware"),
        "Joint '%s' must expose position and velocity command interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto & interface : joint.state_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      } else {
        RCLCPP_FATAL(
          rclcpp::get_logger("MyRobotSystemHardware"),
          "Joint '%s' has unsupported state interface '%s'.",
          joint.name.c_str(), interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    if (!has_position_state || !has_velocity_state)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MyRobotSystemHardware"),
        "Joint '%s' must expose position and velocity state interfaces.",
        joint.name.c_str());
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
      RCLCPP_FATAL(rclcpp::get_logger("MyRobotSystemHardware"), "Joint '%s' has no min/max limits specified in URDF!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    catch (const std::invalid_argument &ex)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MyRobotSystemHardware"), "Joint '%s' min/max limits are not valid numbers!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 읽어온 제한값을 로그로 출력하여 확인
    RCLCPP_INFO(
      rclcpp::get_logger("MyRobotSystemHardware"), "Joint '%s' limits loaded: min=%.3f, max=%.3f",
      joint.name.c_str(), hw_joint_limits_[i].min_position, hw_joint_limits_[i].max_position);
  }

  auto bool_param = [&](const std::string & key, bool default_value) {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      return default_value;
    }
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return (value == "1" || value == "true" || value == "yes" || value == "on");
  };
  auto int_param = [&](const std::string & key, int default_value) {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      return default_value;
    }
    try {
      return std::stoi(it->second);
    } catch (...) {
      return default_value;
    }
  };
  auto float_param = [&](const std::string & key, float default_value) {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) {
      return default_value;
    }
    try {
      return std::stof(it->second);
    } catch (...) {
      return default_value;
    }
  };

  use_hall_homing_ = bool_param("use_hall_homing", use_hall_homing_);
  hall_homing_joints_count_ = int_param("hall_homing_joints_count", hall_homing_joints_count_);
  hall_trigger_state_ = int_param("hall_trigger_state", hall_trigger_state_);
  hall_homing_velocity_ = float_param("hall_homing_velocity", hall_homing_velocity_);
  hall_reapproach_velocity_ = float_param("hall_reapproach_velocity", hall_reapproach_velocity_);
  hall_backoff_duration_sec_ = float_param("hall_backoff_duration_sec", hall_backoff_duration_sec_);
  hall_joint_timeout_sec_ = float_param("hall_joint_timeout_sec", hall_joint_timeout_sec_);
  velocity_ = float_param("trajectory_velocity_rpm", velocity_);
  acceleration_ = float_param("trajectory_acceleration_rpm_s", acceleration_);
  velocity_feedforward_scale_ =
    float_param("velocity_feedforward_scale", velocity_feedforward_scale_);
  final_catchup_velocity_rpm_ =
    float_param("final_catchup_velocity_rpm", final_catchup_velocity_rpm_);
  position_hold_velocity_rpm_ =
    float_param("position_hold_velocity_rpm", position_hold_velocity_rpm_);
  trajectory_catchup_time_sec_ =
    float_param("trajectory_catchup_time_sec", trajectory_catchup_time_sec_);
  trajectory_command_rate_hz_ =
    float_param("trajectory_command_rate_hz", trajectory_command_rate_hz_);
  trajectory_command_rate_hz_ =
    std::clamp(trajectory_command_rate_hz_, 1.0f, 200.0f);
  can_driver.set_position_command_rate_hz(trajectory_command_rate_hz_);
  can_logging_enabled_on_start_ = bool_param("can_logging_enabled_on_start", can_logging_enabled_on_start_);

  auto direction_it = info_.hardware_parameters.find("hall_homing_directions");
  if (direction_it != info_.hardware_parameters.end()) {
    std::stringstream ss(direction_it->second);
    std::string token;
    size_t idx = 0;
    while (std::getline(ss, token, ',') && idx < hall_homing_directions_.size()) {
      try {
        int dir = std::stoi(token);
        hall_homing_directions_[idx] = (dir >= 0) ? 1 : -1;
      } catch (...) {
        hall_homing_directions_[idx] = 1;
      }
      ++idx;
    }
  }

  auto offsets_it = info_.hardware_parameters.find("hall_home_offsets_deg");
  if (offsets_it != info_.hardware_parameters.end()) {
    std::stringstream ss(offsets_it->second);
    std::string token;
    size_t idx = 0;
    while (std::getline(ss, token, ',') && idx < hall_home_offsets_deg_.size()) {
      try {
        hall_home_offsets_deg_[idx] = std::stof(token);
      } catch (...) {
        hall_home_offsets_deg_[idx] = 0.0f;
      }
      ++idx;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotSystemHardware"),
    "Hall homing config: enabled=%s trigger=%d vel=%.3f reapproach=%.3f backoff=%.2fs timeout=%.1fs",
    use_hall_homing_ ? "true" : "false",
    hall_trigger_state_,
    hall_homing_velocity_,
    hall_reapproach_velocity_,
    hall_backoff_duration_sec_,
    hall_joint_timeout_sec_);
  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotSystemHardware"),
    "Trajectory motor command: max_velocity=%.3f RPM acceleration=%.3f RPM/s "
    "feedforward_scale=%.2f min_catchup=%.3f RPM hold=%.3f RPM "
    "catchup_time=%.2fs command_rate=%.1fHz",
    velocity_,
    acceleration_,
    velocity_feedforward_scale_,
    final_catchup_velocity_rpm_,
    position_hold_velocity_rpm_,
    trajectory_catchup_time_sec_,
    trajectory_command_rate_hz_);
  RCLCPP_INFO(
    rclcpp::get_logger("MyRobotSystemHardware"),
    "CAN CSV logging on start: %s",
    can_logging_enabled_on_start_ ? "true" : "false");

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
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]));
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
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]));
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
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Activating ...please wait...");

    set_system_state(SystemState::IDLE);
    j2j4_stopper_origin_done_ = false;

    if (!can_driver.connected()) {
        can_driver.connect();
    }
    if (!can_driver.connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to connect to CAN bus");
        set_system_state(SystemState::ERROR);
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (can_logging_enabled_on_start_ && !can_driver.set_can_logging(true)) {
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to start CAN CSV logging");
    }

    // ── 0) node_/구독자/executor 먼저 생성 → DDS 발견 시간 확보 ──
    try {
      node_ = std::make_shared<rclcpp::Node>("teaching_mode_controller");
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      teaching_mode_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/teaching_mode", 10,
          std::bind(&MyRobotSystemHardware::teaching_mode_callback, this, std::placeholders::_1));
      hall_states_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
          "/hall_states", 10,
          std::bind(&MyRobotSystemHardware::hall_states_callback, this, std::placeholders::_1));
      manual_origin_cmd_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
          "/manual_origin/set_origin_joints", 10,
          std::bind(&MyRobotSystemHardware::manual_origin_command_callback, this, std::placeholders::_1));
      j2j4_free_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/j2j4_origin/free", 10,
          std::bind(&MyRobotSystemHardware::j2j4_free_callback, this, std::placeholders::_1));
      bed_teach_free_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
          "/bed_teach/free_drive", 10,
          std::bind(&MyRobotSystemHardware::bed_teach_free_callback, this, std::placeholders::_1));

      // confirm 구독을 여기서 등록 → 모터 대기 중에 DDS 발견 완료됨
      j2j4_stopper_confirmed_ = false;
      j2j4_confirm_sub_ = node_->create_subscription<std_msgs::msg::Empty>(
          "/j2j4_origin/confirm_stopper", 10,
          [this](const std_msgs::msg::Empty::SharedPtr) {
              j2j4_stopper_confirmed_ = true;
          });

      can_logging_set_srv_ = node_->create_service<std_srvs::srv::SetBool>(
          "/can_logging/set_enabled",
          std::bind(&MyRobotSystemHardware::set_can_logging_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
      can_logging_status_srv_ = node_->create_service<std_srvs::srv::Trigger>(
          "/can_logging/status",
          std::bind(&MyRobotSystemHardware::get_can_logging_status_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
      manual_origin_j2_srv_ = node_->create_service<std_srvs::srv::Trigger>(
          "/manual_origin/set_j2_origin",
          std::bind(&MyRobotSystemHardware::set_manual_origin_j2_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
      manual_origin_j4_srv_ = node_->create_service<std_srvs::srv::Trigger>(
          "/manual_origin/set_j4_origin",
          std::bind(&MyRobotSystemHardware::set_manual_origin_j4_callback, this,
                    std::placeholders::_1, std::placeholders::_2));
      manual_origin_j2_j4_srv_ = node_->create_service<std_srvs::srv::Trigger>(
          "/manual_origin/set_j2_j4_origin",
          std::bind(&MyRobotSystemHardware::set_manual_origin_j2_j4_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

      executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);
      executor_thread_ = std::thread([this]() { executor_->spin(); });

      RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "✅ ROS2 node & subscriptions ready (DDS discovery in progress...)");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to create ROS2 node: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // ── 1) 모터 응답 대기 (J2/J4는 처음부터 duty 0%) ───────────
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Waiting for motors to be ready...");
    command_non_j2j4_hold(can_driver);
    command_j2j4_free_duty(can_driver);

    auto wait_start = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - wait_start).count() < 15.0) {
        int ready = 0;
        for (uint8_t i = 1; i <= 6; i++) {
            if (can_driver.getMotorData(i).motor_id != 0) ready++;
        }
        if (ready >= 6) break;
        command_non_j2j4_hold(can_driver);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Motors responding. Sending set_origin...");

    // ── 2) J1/J3/J5/J6 즉시 set_origin ──────────────────────────
    for (uint8_t i : {1, 3, 5, 6}) {
        can_driver.write_set_origin(i, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        can_driver.write_velocity(i, 0.0f);
    }
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "J1/J3/J5/J6 set_origin 완료");

    // ── 3) J2/J4 free + Confirm 대기 ─────────────────────────────
    command_j2j4_free_duty(can_driver);
    RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
        "=== J2/J4 FREE: duty 0%% test === 손으로 스토퍼에 밀고 런처 [Confirm J2+J4 Stopper] 버튼을 누르세요 (90초)");

    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(90);
        while (!j2j4_stopper_confirmed_ && std::chrono::steady_clock::now() < deadline) {
            command_non_j2j4_hold(can_driver);
            command_j2j4_free_duty(can_driver);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    if (!j2j4_stopper_confirmed_) {
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
            "J2/J4 confirm 타임아웃 — 현재 위치를 스토퍼로 간주");
    }

    // ── 4) 스토퍼 위치 기준 +95도 이동 ──────────────────────────
    constexpr float  kMoveRpm  = 2.0f;
    constexpr float  kMoveAcc  = 5.0f;
    constexpr double k95Deg    = 95.0;
    const double j2_stopper = can_driver.getMotorData(2).position;
    const double j4_stopper = can_driver.getMotorData(4).position;
    const double j2_target  = j2_stopper + k95Deg;
    const double j4_target  = j4_stopper + k95Deg;
    RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
        "J2: stopper=%.2f° → target=%.2f°  |  J4: stopper=%.2f° → target=%.2f°",
        j2_stopper, j2_target, j4_stopper, j4_target);

    can_driver.write_position_velocity(2, j2_target, kMoveRpm, kMoveAcc);
    can_driver.write_position_velocity(4, j4_target, kMoveRpm, kMoveAcc);

    auto mv_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    int stable_count = 0;
    while (std::chrono::steady_clock::now() < mv_deadline) {
        const auto j2_data = can_driver.getMotorData(2);
        const auto j4_data = can_driver.getMotorData(4);
        const double e2 = std::abs(j2_data.position - j2_target);
        const double e4 = std::abs(j4_data.position - j4_target);
        const bool stopped =
            e2 < 0.3 && e4 < 0.3 &&
            std::abs(j2_data.speed) < 0.2 && std::abs(j4_data.speed) < 0.2;
        stable_count = stopped ? stable_count + 1 : 0;
        if (stable_count >= 5) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // ── 5) J2/J4 set_origin ──────────────────────────────────────
    can_driver.write_set_origin(2, false);
    can_driver.write_set_origin(4, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    can_driver.write_velocity(2, 0.0f);
    can_driver.write_velocity(4, 0.0f);
    j2j4_stopper_origin_done_ = true;
    RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "J2/J4 set_origin 완료 → 전체 0,0,0,0,0,0");

    for (uint8_t i = 1; i <= 6; i++) can_driver.write_velocity(i, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "set_origin complete.");

    // 현재 위치를 cmd_에 복사
    for (uint8_t i = 1; i < 7; i++) {
        motor_data = can_driver.getMotorData(i);
        pos_[i-1] = motor_data.position * M_PI / 180.0;
        cmd_[i-1] = pos_[i-1];
        vel_[i-1] = motor_data.speed * 2.0 * M_PI / 60.0;
        cmd_vel_[i-1] = 0.0;
        prev_cmd_[i-1] = cmd_[i-1];
    }
    have_prev_cmd_ = true;
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "✅ Initial cmd_ set to current position:");
    for (uint i = 0; i < 6; ++i) {
        RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "  Joint %d: cmd=%.3f, pos=%.3f",
                   i+1, cmd_[i], pos_[i]);
    }

    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "✅ CAN logging services ready: /can_logging/set_enabled, /can_logging/status");

    if (use_hall_homing_) {
      set_system_state(SystemState::HOMING);
      start_hall_homing();
    } else {
      // No homing required, go directly to READY
      set_system_state(SystemState::READY);
    }
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Successfully activated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn MyRobotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Deactivating ...please wait...");
  // ✅ 새로 추가: 교시모드 종료
  if (teaching_mode_active_) {
      stop_teaching_mode();
  }
  // ✅ Executor 및 노드 정리
  if (executor_) {
      executor_->cancel();
  }
  if (executor_thread_.joinable()) {
      executor_thread_.join();
  }
  if (node_) {
      node_.reset();
  }
  hall_homing_active_ = false;
  hall_homing_failed_ = false;
  //comms_.disconnect(); // 연결 끊기
  can_driver.disconnect();
  RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Successfully deactivated!");
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
      raw_pos_[i-1] = motor_data.position * M_PI / 180.0;
      pos_[i-1] = raw_pos_[i-1] + joint_home_offsets_[i-1];
      spd_[i-1] = motor_data.speed;
      vel_[i-1] = static_cast<double>(spd_[i-1]) * 2.0 * M_PI / 60.0;
  }
  // 교시모드일 때 데이터 수집 (100Hz)
  if (teaching_mode_active_) {
    // 속도와 전류 데이터도 함께 수집
    float joint_velocities[6];
    float joint_currents[6];
    for (int i = 0; i < 6; i++) {
            joint_velocities[i] = spd_[i];  // RPM 단위
            auto motor_data_current = can_driver.getMotorData(i + 1);
            joint_currents[i] = motor_data_current.current;  // Ampere 단위
    }
    // End-Effector Pose 계산
    double ee_x = 0.0, ee_y = 0.0, ee_z = 0.0;
    double ee_roll = 0.0, ee_pitch = 0.0, ee_yaw = 0.0;

    if (get_end_effector_pose(ee_x, ee_y, ee_z, ee_roll, ee_pitch, ee_yaw)) {
          teaching_logger_.log_frame(pos_, joint_velocities, joint_currents,
                                     ee_x, ee_y, ee_z, ee_roll, ee_pitch, ee_yaw);
    } else {
          // tf2 조회 실패 시 0으로 기록 (또는 이전 값 유지)
          teaching_logger_.log_frame(pos_, joint_velocities, joint_currents,
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!can_driver.connected()) {
    set_system_state(SystemState::ERROR);
    return hardware_interface::return_type::ERROR;
  }

  try {
    auto current_state = get_system_state();

    // ERROR state: stop all motors and refuse commands
    if (current_state == SystemState::ERROR) {
      emergency_stop_all_motors();
      return hardware_interface::return_type::ERROR;
    }

    // HOMING state: run homing step
    if (hall_homing_active_) {
      if (!run_hall_homing_step()) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::OK;
    }

    // J2/J4 stopper-based origin homing (teaching 모드와 독립)
    if (j2j4_origin_homing_active_) {
      run_j2j4_stopper_homing_step();
      return hardware_interface::return_type::OK;
    }

    // Bed teaching free-drive: all joints duty 0%, no position hold.
    if (bed_teach_free_active_) {
      command_all_free_duty(can_driver);
      return hardware_interface::return_type::OK;
    }

    // 교시 모드 체크
    if (teaching_mode_active_) {
      can_driver.write_brake_current(1, teaching_brake_currents_[0]);
      can_driver.write_brake_current(2, teaching_brake_currents_[1]);
      can_driver.write_brake_current(3, teaching_brake_currents_[2]);
      can_driver.write_brake_current(4, teaching_brake_currents_[3]);
      can_driver.write_brake_current(5, teaching_brake_currents_[4]);
      can_driver.write_brake_current(6, teaching_brake_currents_[5]);
      return hardware_interface::return_type::OK;
    } else {
      // 일반 모드: READY 또는 RUNNING 상태
      if (current_state != SystemState::READY && current_state != SystemState::RUNNING) {
        return hardware_interface::return_type::OK;
      }

      // Check if trajectory is being executed (cmd_ changed from current pos_)
      const double POSITION_THRESHOLD = 0.01; // 0.01 rad (~0.57 deg)
      bool motion_in_progress = false;
      for (uint i = 0; i < 6; ++i) {
        if (std::abs(cmd_[i] - pos_[i]) > POSITION_THRESHOLD) {
          motion_in_progress = true;
          break;
        }
      }

      // State transition: READY → RUNNING when new trajectory received
      if (current_state == SystemState::READY && motion_in_progress) {
        ready_to_running_debounce_ = 0;
        set_system_state(SystemState::RUNNING);
      }
      // State transition: RUNNING → READY when motion completed (debounce 50 cycles)
      else if (current_state == SystemState::RUNNING && !motion_in_progress) {
        ready_to_running_debounce_++;
        if (ready_to_running_debounce_ >= 50) {
          ready_to_running_debounce_ = 0;
          set_system_state(SystemState::READY);
        }
      } else {
        ready_to_running_debounce_ = 0;
      }

      const float limited_acceleration = acceleration_;
      (void)period;
      constexpr double kRpmPerRadPerSec = 60.0 / (2.0 * M_PI);

      // J2/J4 free mode: duty 0%로 손 이동 가능 여부 실험
      if (j2j4_free_active_) {
        command_j2j4_free_duty(can_driver);
      }

      // for 루프를 사용하여 모든 조인트에 제한을 적용하고 명령을 전송합니다.
      for (uint i =0; i < 6; ++i)
      {
          // J2/J4 free mode 중이면 해당 관절 포지션 명령 스킵
          if (j2j4_free_active_ && (i == 1 || i == 3)) continue;
          // 컨트롤러부터 받은 명령에 조인트 제한을 적용합니다.
          double clamped_command_rad = std::clamp(
            cmd_[i],
            hw_joint_limits_[i].min_position,
            hw_joint_limits_[i].max_position
          );
          // 조인트 제한을 적용한 명령을 모터가 사용하는 단위로 변환한다
          double raw_target_rad = clamped_command_rad - joint_home_offsets_[i];
          double command_deg = raw_target_rad * 180.0 / M_PI; // radian to degree 변환

          // Preserve the per-axis velocity ratio from the timed JTC trajectory.
          // Position error feedback only raises lagging axes enough to catch up.
          const double commanded_velocity_rad_s =
            std::isfinite(cmd_vel_[i]) ? std::abs(cmd_vel_[i]) : 0.0;
          double motor_velocity_rpm =
            commanded_velocity_rad_s * kRpmPerRadPerSec *
            velocity_feedforward_scale_;

          const double position_error_rad =
            std::abs(clamped_command_rad - pos_[i]);
          const bool has_position_error = position_error_rad > 1.0e-3;
          if (has_position_error) {
            const double catchup_time =
              std::max(static_cast<double>(trajectory_catchup_time_sec_), 0.1);
            const double catchup_velocity_rpm =
              position_error_rad / catchup_time * kRpmPerRadPerSec;
            motor_velocity_rpm = std::max(
              motor_velocity_rpm,
              std::max(
                catchup_velocity_rpm,
                static_cast<double>(final_catchup_velocity_rpm_)));
          } else if (motor_velocity_rpm < 1.0e-6) {
            motor_velocity_rpm = position_hold_velocity_rpm_;
          }
          motor_velocity_rpm = std::clamp(
            motor_velocity_rpm, 0.0, static_cast<double>(velocity_));

          // 최종 값읋 CAN 드라이버에 전송한다 (모터 ID는 1부터 시작하므로 i+1 사용)
          can_driver.write_position_velocity(
            i + 1,
            command_deg,
            static_cast<float>(motor_velocity_rpm),
            limited_acceleration);
          prev_cmd_[i] = clamped_command_rad;
      }
      have_prev_cmd_ = true;
    }
    
    // 디버그 출력 (필요시)
    /*
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0) {  // 1초마다 출력
        RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Velocity: %.1f RPM, Acceleration: %.1f RPM/s",
                   limited_velocity, limited_acceleration);
    }*/
  }
  catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Failed to write command: %s", e.what());
      return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}


// 사용 방법:
// ros2 topic pub /teaching_mode std_msgs/msg/Bool "{data: true}" --once 시작
// ros2 topic pub /teaching_mode std_msgs/msg/Bool "{data: false}" --once 종료

// # 토픽 리스트 확인
// ros2 topic list | grep teaching

// # 현재 교시모드 상태 확인 (로그 확인)
// ros2 topic echo /rosout
// ✅ 토픽 콜백 함수
void MyRobotSystemHardware::teaching_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !teaching_mode_active_) {
        start_teaching_mode();
    } else if (!msg->data && teaching_mode_active_) {
        stop_teaching_mode();
    }
}

void MyRobotSystemHardware::set_can_logging_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    const bool ok = can_driver.set_can_logging(request->data);
    const bool enabled = can_driver.is_can_logging();
    const std::string filename = can_driver.get_can_log_filename();

    response->success = ok;
    std::ostringstream message;
    message << "CAN logging " << (enabled ? "ON" : "OFF");
    if (!filename.empty()) {
        message << " file=" << filename;
    }
    response->message = message.str();

    RCLCPP_INFO(
        rclcpp::get_logger("MyRobotSystemHardware"),
        "%s",
        response->message.c_str());
}

void MyRobotSystemHardware::get_can_logging_status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    const bool enabled = can_driver.is_can_logging();
    const std::string filename = can_driver.get_can_log_filename();

    response->success = enabled;
    std::ostringstream message;
    message << "CAN logging " << (enabled ? "ON" : "OFF");
    if (!filename.empty()) {
        message << " file=" << filename;
    }
    response->message = message.str();
}

bool MyRobotSystemHardware::set_manual_joint_origin(size_t joint_index, std::string & message)
{
    if (joint_index >= 6) {
        message = "Invalid joint index";
        return false;
    }
    if (hall_homing_active_) {
        message = "Refusing manual origin while Hall homing is active";
        return false;
    }
    if (!can_driver.connected()) {
        message = "CAN driver is not connected";
        return false;
    }

    const uint8_t motor_id = static_cast<uint8_t>(joint_index + 1);
    MotorData data = can_driver.getMotorData(motor_id);
    if (data.motor_id == 0) {
        message = "No motor feedback for J" + std::to_string(joint_index + 1);
        return false;
    }
    if (std::abs(data.speed) > 0.3f) {
        std::ostringstream oss;
        oss << "Refusing manual origin for J" << (joint_index + 1)
            << " because motor is still moving (" << data.speed << " RPM)";
        message = oss.str();
        return false;
    }

    // Operator physically positions the arm at stopper+95deg, then calls this.
    // write_set_origin makes the current position (stopper+95deg) the motor's raw zero.
    // No home offset needed — the motor's internal zero IS the logical zero.
    can_driver.write_velocity(motor_id, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    can_driver.write_set_origin(motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    can_driver.write_velocity(motor_id, 0.0f);

    raw_pos_[joint_index] = 0.0;
    joint_home_offsets_[joint_index] = 0.0;
    pos_[joint_index] = 0.0;
    cmd_[joint_index] = 0.0;
    cmd_vel_[joint_index] = 0.0;
    vel_[joint_index] = 0.0;
    spd_[joint_index] = 0.0f;
    prev_cmd_[joint_index] = 0.0;
    joint_homed_[joint_index] = true;

    std::ostringstream oss;
    oss << "Manual origin set for J" << (joint_index + 1)
        << " using CubeMars set_origin(false)";
    RCLCPP_WARN(
        rclcpp::get_logger("MyRobotSystemHardware"),
        "%s",
        message.c_str());
    return true;
}

void MyRobotSystemHardware::set_manual_origin_j2_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = set_manual_joint_origin(1, response->message);
}

void MyRobotSystemHardware::set_manual_origin_j4_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    response->success = set_manual_joint_origin(3, response->message);
}

void MyRobotSystemHardware::set_manual_origin_j2_j4_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    std::string j2_message;
    std::string j4_message;
    const bool j2_ok = set_manual_joint_origin(1, j2_message);
    const bool j4_ok = j2_ok && set_manual_joint_origin(3, j4_message);

    response->success = j2_ok && j4_ok;
    response->message = j2_ok ? (j2_message + "; " + j4_message) : j2_message;
}

void MyRobotSystemHardware::manual_origin_command_callback(
    const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.empty()) {
        RCLCPP_WARN(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "Manual origin topic ignored: empty joint list");
        return;
    }

    if (hall_homing_active_) {
        RCLCPP_ERROR(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "J2J4 origin homing rejected: Hall homing 진행 중");
        return;
    }

    // data에 2 또는 4가 있으면 J2/J4 stopper homing 시작
    bool want_j2j4 = false;
    for (const int jn : msg->data) {
        if (jn == 2 || jn == 4) { want_j2j4 = true; break; }
    }
    if (!want_j2j4) {
        RCLCPP_WARN(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "J2J4 origin homing: J2/J4 번호가 없음, 무시");
        return;
    }

    start_j2j4_stopper_homing();
}

void MyRobotSystemHardware::start_j2j4_stopper_homing()
{
    if (j2j4_origin_homing_active_) {
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
            "J2J4 origin homing already active, ignoring");
        return;
    }
    j2j4_free_active_ = false;  // free mode 해제

    constexpr size_t kJoints[2] = {1, 3};
    for (int idx = 0; idx < 2; ++idx) {
        j2j4_zero_target_[idx]  = pos_[kJoints[idx]] + kJ2J4ZeroOffsetRad;
        j2j4_arrive_count_[idx] = 0;
    }
    j2j4_origin_homing_active_ = true;

    RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
        "J2J4 origin: J2=%.3f→%.3f rad, J4=%.3f→%.3f rad (+95deg)",
        pos_[1], j2j4_zero_target_[0],
        pos_[3], j2j4_zero_target_[1]);
}

void MyRobotSystemHardware::run_j2j4_stopper_homing_step()
{
    constexpr size_t kJoints[2] = {1, 3};
    bool all_arrived = true;

    for (int idx = 0; idx < 2; ++idx) {
        const size_t j = kJoints[idx];
        const double raw_target_rad = j2j4_zero_target_[idx] - joint_home_offsets_[j];
        const double command_deg    = raw_target_rad * 180.0 / M_PI;
        can_driver.write_position_velocity(j + 1, command_deg, kJ2J4ZeroMoveRpm, acceleration_);

        const double err = std::abs(pos_[j] - j2j4_zero_target_[idx]);
        if (err < kJ2J4ArriveThreshRad) {
            j2j4_arrive_count_[idx]++;
        } else {
            j2j4_arrive_count_[idx] = 0;
            all_arrived = false;
        }
        if (j2j4_arrive_count_[idx] < kJ2J4ArriveDebounce) all_arrived = false;
    }

    if (all_arrived) {
        for (int idx = 0; idx < 2; ++idx) {
            apply_origin_in_write(kJoints[idx]);
        }
        j2j4_origin_homing_active_ = false;
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
            "J2J4 origin homing complete: J2/J4 원점 설정 완료");
    }
}

void MyRobotSystemHardware::j2j4_free_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    j2j4_free_active_ = msg->data;
    if (j2j4_free_active_) {
        // free 모드 진입 시 J2/J4 cmd를 현재 위치로 고정 (복귀 방지)
        cmd_[1] = pos_[1];
        cmd_[3] = pos_[3];
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
            "J2/J4 free mode ON: duty 0%% command active");
    } else {
        RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"),
            "J2/J4 free mode OFF");
    }
}

void MyRobotSystemHardware::bed_teach_free_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    bed_teach_free_active_ = msg->data;

    if (bed_teach_free_active_) {
        teaching_mode_active_ = false;
        j2j4_free_active_ = false;
        for (size_t i = 0; i < 6; ++i) {
            cmd_[i] = pos_[i];
            cmd_vel_[i] = 0.0;
            prev_cmd_[i] = pos_[i];
        }
        RCLCPP_WARN(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "Bed teach free-drive ON: all joints duty 0%%. Support the arm before moving.");
    } else {
        for (size_t i = 0; i < 6; ++i) {
            cmd_[i] = pos_[i];
            cmd_vel_[i] = 0.0;
            prev_cmd_[i] = pos_[i];
        }
        RCLCPP_WARN(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "Bed teach free-drive OFF: command synchronized to current joint positions.");
    }
}

void MyRobotSystemHardware::apply_origin_in_write(size_t joint_index)
{
    const uint8_t motor_id = static_cast<uint8_t>(joint_index + 1);
    can_driver.write_set_origin(motor_id, false);
    raw_pos_[joint_index]          = 0.0;
    joint_home_offsets_[joint_index] = 0.0;
    pos_[joint_index]              = 0.0;
    cmd_[joint_index]              = 0.0;
    cmd_vel_[joint_index]          = 0.0;
    vel_[joint_index]              = 0.0;
    spd_[joint_index]              = 0.0f;
    prev_cmd_[joint_index]         = 0.0;
    joint_homed_[joint_index]      = true;
}

// ✅ 교시모드 시작
void MyRobotSystemHardware::start_teaching_mode()
{
    if (teaching_mode_active_) {
        return;
    }
    
    // 안전성 검사
    if (!validate_teaching_safety()) {
        RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "❌ Teaching mode safety check failed!");
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "🔴 Starting teaching mode...");
    
    // 교시 데이터 로깅 시작
    if (!teaching_logger_.start_teaching("robot_teaching")) {
        RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "❌ Failed to start teaching logger!");
        return;
    }
    
    teaching_mode_active_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "✅ Teaching mode started! Manually move the robot...");
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "💡 Brake currents: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] A",
               teaching_brake_currents_[0], teaching_brake_currents_[1], 
               teaching_brake_currents_[2], teaching_brake_currents_[3], 
               teaching_brake_currents_[4], teaching_brake_currents_[5]);
}

// ✅ 교시모드 중지
void MyRobotSystemHardware::stop_teaching_mode()
{
    if (!teaching_mode_active_) {
        return;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "⏹️  Stopping teaching mode...");
    
    teaching_mode_active_ = false;
    teaching_logger_.stop_teaching();
    
    // 모든 모터를 안전하게 정지
    for (uint i = 0; i < 6; ++i) {
        try {
            can_driver.write_velocity(i + 1, 0.0f);
        } catch (const std::exception& e) {
            RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "Warning: Failed to stop motor %d: %s", i+1, e.what());
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "✅ Teaching mode stopped!");
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "💡 Use topic to start new teaching session");
}

void MyRobotSystemHardware::hall_states_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 6) {
        return;
    }
    std::lock_guard<std::mutex> lock(hall_mutex_);
    for (size_t i = 0; i < 6; ++i) {
        hall_states_[i] = msg->data[i];
    }
    hall_data_received_ = true;
}

bool MyRobotSystemHardware::is_hall_triggered(size_t joint_index)
{
    std::lock_guard<std::mutex> lock(hall_mutex_);
    if (joint_index >= hall_states_.size()) {
        return false;
    }
    return hall_states_[joint_index] == hall_trigger_state_;
}

void MyRobotSystemHardware::start_hall_homing()
{
    hall_homing_active_ = true;
    hall_homing_failed_ = false;
    active_homing_joint_ = 0;
    hall_homing_phase_ = HallHomingPhase::APPROACH;
    hall_phase_start_time_ = std::chrono::steady_clock::now();
    hall_joint_start_time_ = hall_phase_start_time_;
    joint_homed_.fill(false);
    joint_home_offsets_.fill(0.0);
    RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "Hall homing started (J1 -> J6)");
}

void MyRobotSystemHardware::finish_joint_homing()
{
    const size_t j = active_homing_joint_;
    const float offset_deg = hall_home_offsets_deg_[j];

    if (std::abs(offset_deg) > 0.1f) {
        // 오프셋 있음: 트리거 위치 저장 후 오프셋 위치로 이동
        trigger_raw_pos_ = raw_pos_[j];
        float target_deg = static_cast<float>(trigger_raw_pos_ * 180.0 / M_PI) + offset_deg;
        RCLCPP_INFO(
            rclcpp::get_logger("MyRobotSystemHardware"),
            "Joint %zu: trigger=%.2f deg, moving to home (offset=%.1f deg, target=%.2f deg)",
            j + 1, trigger_raw_pos_ * 180.0 / M_PI, offset_deg, target_deg);
        can_driver.write_position_velocity(static_cast<uint8_t>(j + 1), target_deg, 2.0f, 5.0f);
        hall_homing_phase_ = HallHomingPhase::MOVING_TO_HOME;
        hall_phase_start_time_ = std::chrono::steady_clock::now();
        return;
    }

    // 오프셋 없음: 트리거 위치 = 0도
    joint_home_offsets_[j] = -raw_pos_[j];
    pos_[j] = 0.0;
    cmd_[j] = 0.0;
    joint_homed_[j] = true;
    can_driver.write_velocity(static_cast<uint8_t>(j + 1), 0.0f);
    RCLCPP_INFO(
        rclcpp::get_logger("MyRobotSystemHardware"),
        "✅ Joint %zu homed. offset(rad)=%.6f", j + 1, joint_home_offsets_[j]);
    advance_to_next_joint();
}

void MyRobotSystemHardware::advance_to_next_joint()
{
    while (true) {
        active_homing_joint_++;

        // J2/J4는 stopper+95deg에서 CubeMars 내부 원점을 이미 잡았으면 Hall homing으로 덮지 않는다.
        if (j2j4_stopper_origin_done_ &&
            (active_homing_joint_ == 1 || active_homing_joint_ == 3)) {
            joint_homed_[active_homing_joint_] = true;
            joint_home_offsets_[active_homing_joint_] = 0.0;
            pos_[active_homing_joint_] = 0.0;
            cmd_[active_homing_joint_] = 0.0;
            RCLCPP_INFO(
                rclcpp::get_logger("MyRobotSystemHardware"),
                "⏭ Joint %zu skipped (J2/J4 stopper origin already set)",
                active_homing_joint_ + 1);
            continue;
        }

        // J6(인덱스 5)은 그리퍼이므로 호밍 스킵
        if (active_homing_joint_ == 5) {
            joint_homed_[5] = true;
            joint_home_offsets_[5] = 0.0;
            pos_[5] = 0.0;
            cmd_[5] = 0.0;
            RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"), "⏭ Joint 6 skipped (gripper)");
            continue;
        }

        break;
    }

    if (active_homing_joint_ >= static_cast<size_t>(hall_homing_joints_count_) ||
        active_homing_joint_ >= 6) {
        hall_homing_active_ = false;
        for (size_t i = 0; i < 6; ++i) cmd_[i] = pos_[i];
        RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"),
            "✅ Hall homing completed (%d/%d joints)",
            static_cast<int>(active_homing_joint_), hall_homing_joints_count_);
        set_system_state(SystemState::READY);
        return;
    }
    hall_homing_phase_ = HallHomingPhase::APPROACH;
    hall_phase_start_time_ = std::chrono::steady_clock::now();
    hall_joint_start_time_ = hall_phase_start_time_;
}

void MyRobotSystemHardware::fail_hall_homing(const std::string & reason)
{
    hall_homing_failed_ = true;
    hall_homing_active_ = false;
    for (uint8_t i = 1; i <= 6; ++i) {
        try {
            can_driver.write_velocity(i, 0.0f);
        } catch (...) {
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "❌ Hall homing failed: %s", reason.c_str());
    set_system_state(SystemState::ERROR);
}

bool MyRobotSystemHardware::run_hall_homing_step()
{
    const auto now = std::chrono::steady_clock::now();

    if (!hall_data_received_) {
        // 데이터 미수신 상태에서도 모터는 계속 velocity 명령 유지
        const size_t j = active_homing_joint_;
        const int dir = hall_homing_directions_[j];
        const uint8_t motor_id = static_cast<uint8_t>(j + 1);
        if (hall_homing_phase_ == HallHomingPhase::APPROACH) {
            can_driver.write_velocity(motor_id, dir * hall_homing_velocity_);
        }
        return true;
    }
    const double joint_elapsed = std::chrono::duration<double>(now - hall_joint_start_time_).count();
    if (joint_elapsed > hall_joint_timeout_sec_) {
        const size_t j_timeout = active_homing_joint_;
        const char* phase_str = (hall_homing_phase_ == HallHomingPhase::APPROACH)      ? "APPROACH" :
                                (hall_homing_phase_ == HallHomingPhase::BACKOFF)     ? "BACKOFF"  :
                                (hall_homing_phase_ == HallHomingPhase::REAPPROACH)  ? "REAPPROACH" :
                                                                                       "MOVING_TO_HOME";
        std::string reason =
            "Joint " + std::to_string(j_timeout + 1) +
            " timeout: no Hall trigger in " + std::to_string(static_cast<int>(hall_joint_timeout_sec_)) + "s" +
            " (phase=" + phase_str +
            ", hall_state=" + std::to_string(hall_states_[j_timeout]) + ")";
        fail_hall_homing(reason);
        return false;
    }

    const size_t j = active_homing_joint_;
    const int dir = hall_homing_directions_[j];
    const uint8_t motor_id = static_cast<uint8_t>(j + 1);

    // 5초마다 진행상황 로그
    static std::chrono::steady_clock::time_point last_progress_log{};
    if (std::chrono::duration<double>(now - last_progress_log).count() >= 5.0) {
        last_progress_log = now;
        const char* phase_str = (hall_homing_phase_ == HallHomingPhase::APPROACH)     ? "APPROACH" :
                                (hall_homing_phase_ == HallHomingPhase::BACKOFF)      ? "BACKOFF"  :
                                (hall_homing_phase_ == HallHomingPhase::REAPPROACH)   ? "REAPPROACH" :
                                                                                        "MOVING_TO_HOME";
        RCLCPP_INFO(rclcpp::get_logger("MyRobotSystemHardware"),
            "🔍 Homing J%zu [%s] hall=%d pos=%.2f°",
            j + 1, phase_str, hall_states_[j],
            static_cast<float>(raw_pos_[j] * 180.0 / M_PI));
    }

    switch (hall_homing_phase_) {
      case HallHomingPhase::APPROACH:
        can_driver.write_velocity(motor_id, dir * hall_homing_velocity_);
        if (is_hall_triggered(j)) {
            can_driver.write_velocity(motor_id, 0.0f);
            hall_homing_phase_ = HallHomingPhase::BACKOFF;
            hall_phase_start_time_ = now;
        }
        break;
      case HallHomingPhase::BACKOFF: {
        can_driver.write_velocity(motor_id, -dir * hall_homing_velocity_);
        const double backoff_elapsed = std::chrono::duration<double>(now - hall_phase_start_time_).count();
        if (backoff_elapsed >= hall_backoff_duration_sec_) {
            can_driver.write_velocity(motor_id, 0.0f);
            hall_homing_phase_ = HallHomingPhase::REAPPROACH;
        }
        break;
      }
      case HallHomingPhase::REAPPROACH:
        can_driver.write_velocity(motor_id, dir * hall_reapproach_velocity_);
        if (is_hall_triggered(j)) {
            finish_joint_homing();
        }
        break;
      case HallHomingPhase::MOVING_TO_HOME: {
        // 오프셋 위치 도달 확인 (0.5도 이내)
        float target_deg = static_cast<float>(trigger_raw_pos_ * 180.0 / M_PI) + hall_home_offsets_deg_[j];
        float current_deg = static_cast<float>(raw_pos_[j] * 180.0 / M_PI);
        if (std::abs(current_deg - target_deg) < 0.5f) {
            joint_home_offsets_[j] = -raw_pos_[j];
            pos_[j] = 0.0;
            cmd_[j] = 0.0;
            joint_homed_[j] = true;
            can_driver.write_velocity(motor_id, 0.0f);
            RCLCPP_INFO(
                rclcpp::get_logger("MyRobotSystemHardware"),
                "✅ Joint %zu homed at offset. offset(rad)=%.6f", j + 1, joint_home_offsets_[j]);
            advance_to_next_joint();
        }
        break;
      }
    }
    return true;
}

// ✅ End-Effector Pose 계산 (tf2 이용)
bool MyRobotSystemHardware::get_end_effector_pose(
    double& x, double& y, double& z,
    double& roll, double& pitch, double& yaw)
{
    try {
        // base_link에서 end_effector (또는 tool0, link6 등) 까지의 변환 조회
        // ⚠️ 실제 URDF의 end-effector 링크 이름으로 변경 필요
        geometry_msgs::msg::TransformStamped transform_stamped = 
            tf_buffer_->lookupTransform(
                "base_link",           // 베이스 프레임
                "tcp",              // End-effector 프레임 (URDF에 맞게 수정)
                tf2::TimePointZero,    // 최신 데이터 사용
                tf2::durationFromSec(0.1)  // 타임아웃 100ms
            );
        
        // Position 추출
        x = transform_stamped.transform.translation.x;
        y = transform_stamped.transform.translation.y;
        z = transform_stamped.transform.translation.z;
        
        // Orientation (Quaternion -> RPY 변환)
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w
        );
        
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        
        return true;
        
    } catch (const tf2::TransformException& ex) {
        // tf2 조회 실패 (초기화 중이거나 아직 데이터 없음)
        // 너무 자주 출력되면 성능 저하되므로 간헐적으로만 로그
        static int error_count = 0;
        if (++error_count % 100 == 0) {  // 1초마다 한 번씩만 출력
            RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "Could not get end-effector transform: %s", ex.what());
        }
        return false;
    }
}

// ✅ 비상정지
void MyRobotSystemHardware::emergency_stop_all_motors()
{
    RCLCPP_WARN(rclcpp::get_logger("MyRobotSystemHardware"), "🚨 Emergency stop activated!");
    for (uint8_t i = 1; i <= 6; i++) {
        try {
            can_driver.write_velocity(i, 0.0f);
        } catch (...) {
            // 비상정지는 예외가 발생해도 계속 진행
        }
    }
}

// ✅ 안전성 검사
bool MyRobotSystemHardware::validate_teaching_safety()
{
    if (!can_driver.connected()) {
        RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "CAN driver not connected");
        return false;
    }
    
    for (uint8_t i = 1; i <= 6; i++) {
        auto motor_data = can_driver.getMotorData(i);
        if (motor_data.error != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MyRobotSystemHardware"), "Motor %d has error code: %d", i, motor_data.error);
            return false;
        }
    }
    
    return true;
}

}  // namespace my_robot_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_robot_ros2_control::MyRobotSystemHardware, hardware_interface::SystemInterface)
