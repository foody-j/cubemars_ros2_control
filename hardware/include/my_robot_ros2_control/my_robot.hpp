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

#ifndef ROS2_CONTROL__MY_ROBOT_HPP_
#define ROS2_CONTROL__MY_ROBOT_HPP_

#include <memory> // 필요한 C++ 표준 라이브러리 헤더 포함
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string> // 스마트 포인터 사용을 위한 헤더
#include <thread>
#include <vector> // 벡터 컨테이너 사용을 위한 헤더

// ROS 2 하드웨어 인터페이스 관련 헤더 들
#include "hardware_interface/handle.hpp"  // 하드웨어 핸들 정의
#include "hardware_interface/hardware_info.hpp" // 하드웨어 정보 구조체 정의
#include "hardware_interface/system_interface.hpp"  // 시스템 인터페이스 기본 클래스
#include "hardware_interface/types/hardware_interface_return_values.hpp" // 반환값 타입 정의
#include "rclcpp/macros.hpp"  // ROS 2 C++ 매크로 정의
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"  // 생명 주기 노드 정의
#include "rclcpp_lifecycle/state.hpp" // 생명 주기 상태 정의
#include "motor_can_driver.hpp"
#include "motor_data.hpp"
#include "teaching_data_logger.hpp"

// ROS 2 메시지 타입
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

// tf2 관련 헤더
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"


// ROS 2 컨트롤 데모 예제 1의 네임스페이스 시작
namespace my_robot_ros2_control
{

// 시스템 상태머신 정의
enum class SystemState {
    IDLE,      // 초기 상태 (비활성화)
    HOMING,    // Hall 센서 기반 원점복귀 중
    READY,     // 원점복귀 완료, 작업 대기
    RUNNING,   // 궤적 실행 중
    ERROR      // 에러 발생 (비상정지)
};

// 조인트 제한값을 담을 구조체를 정의.
struct JointLimits {
    double min_position{0.0};  // 최소 위치 (라디안)
    double max_position{0.0};  // 최대 위치 (라디안)

};

// RRBot 시스템의 위치 제어만을 위한 하드웨어 인터페이스 클래스
class MyRobotSystemHardware : public hardware_interface::SystemInterface
{

  
public:
  // 공유 포인터 정의를 위한 매크로
  RCLCPP_SHARED_PTR_DEFINITIONS(MyRobotSystemHardware);
  
  // 하드웨어 초기화 콜백 함수
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  // 하드웨어 설정 콜백 함수
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 클린업 콜백 함수
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 활성화 콜백 함수
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 비활성화 콜백 함수
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  // 하드웨어 상태 읽기 함수
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  // 하드웨어 명령 쓰기 함수
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the RRBot simulation
  CanComms can_driver;  // CAN 통신 객체
  MotorDataManager motor_manager_;
  int hw_start_sec_ = 2;  // 또는 원하는 값으로 설정
  double pos_[6] = {0.0f};
  double vel_[6] = {0.0};
  float spd_[6] = {0.0f};
  double cmd_[6] = {0.0};  // 위치 명령을 저장할 배열
  double cmd_vel_[6] = {0.0};
  double prev_cmd_[6] = {0.0};
  bool have_prev_cmd_{false};
  float velocity_{3.0f};      // RPM
  float acceleration_{15.0f};  // RPM/s
  float velocity_feedforward_scale_{1.25f};
  float final_catchup_velocity_rpm_{0.10f};
  float position_hold_velocity_rpm_{0.30f};
  float trajectory_catchup_time_sec_{1.5f};
  float trajectory_command_rate_hz_{100.0f};
  MotorData motor_data;
  // ▼▼▼ 2. JointLimits 구조체 벡터를 멤버 변수로 추가합니다. ▼▼▼
  // 이 변수에 on_init 단계에서 읽어온 각 조인트의 min/max 값이 저장됩니다.
  std::vector<JointLimits> hw_joint_limits_;

  // ✅ ROS2 노드 및 토픽 구독자
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr teaching_mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr hall_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr manual_origin_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr j2j4_free_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bed_teach_free_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr can_logging_set_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr can_logging_status_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_origin_j2_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_origin_j4_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_origin_j2_j4_srv_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;
  
  // ✅ tf2 관련 (End-Effector Pose 계산용)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ✅ 새로 추가: Teaching Mode 관련 멤버 변수들
  std::atomic<bool> teaching_mode_active_{false};
  SimpleTeachingLogger teaching_logger_;
  bool can_logging_enabled_on_start_{true};
  // 교시모드 설정 (사용자가 설정 가능)
  std::array<float, 6> teaching_brake_currents_ = {
      0.0f, 0.0f,  // 관절 1,2: 큰 관절이므로 높은 브레이크 전류
      1.0f, 1.0f,  // 관절 3,4: 중간 크기
      1.5f, 1.5f   // 관절 5,6: 작은 관절이므로 낮은 브레이크 전류
  };

  // System state machine
  SystemState system_state_{SystemState::IDLE};
  mutable std::mutex state_mutex_;
  int ready_to_running_debounce_{0};

  // Hall sensor based homing
  bool use_hall_homing_{true};
  int hall_homing_joints_count_{6};
  int hall_trigger_state_{1};
  float hall_homing_velocity_{0.4f};
  float hall_reapproach_velocity_{0.2f};
  float hall_backoff_duration_sec_{0.4f};
  float hall_joint_timeout_sec_{12.0f};
  std::array<int, 6> hall_homing_directions_{{1, 1, 1, 1, -1, 1}};
  std::array<float, 6> hall_home_offsets_deg_{{0.0f, -100.0f, -180.0f, -100.0f, 0.0f, 0.0f}};
  std::array<int, 6> hall_states_{{0, 0, 0, 0, 0, 0}};
  std::array<bool, 6> joint_homed_{{false, false, false, false, false, false}};
  std::array<double, 6> raw_pos_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  std::array<double, 6> joint_home_offsets_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  double trigger_raw_pos_{0.0};  // 트리거 시점의 raw 위치 저장
  std::atomic<bool> hall_data_received_{false};
  bool hall_homing_active_{false};
  bool hall_homing_failed_{false};
  size_t active_homing_joint_{0};
  enum class HallHomingPhase { APPROACH, BACKOFF, REAPPROACH, MOVING_TO_HOME };
  HallHomingPhase hall_homing_phase_{HallHomingPhase::APPROACH};
  std::chrono::steady_clock::time_point hall_phase_start_time_;
  std::chrono::steady_clock::time_point hall_joint_start_time_;
  std::mutex hall_mutex_;

  // J2/J4 independent stopper-based origin homing (teaching mode 무관)
  bool j2j4_stopper_confirmed_{false};    // on_activate() 중 스토퍼 확인 신호
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr j2j4_confirm_sub_;
  bool j2j4_free_active_{false};          // J2/J4 duty 0%, 손 이동 가능 여부 실험
  bool bed_teach_free_active_{false};      // 4점 베드 교시용 6축 duty 0% free-drive
  bool j2j4_stopper_origin_done_{false};  // J2/J4는 Hall homing 대신 stopper 기준 원점 사용
  bool j2j4_origin_homing_active_{false};
  std::array<double, 2> j2j4_zero_target_{{0.0, 0.0}};
  std::array<int, 2>    j2j4_arrive_count_{{0, 0}};
  static constexpr float   kJ2J4ZeroMoveRpm    = 2.0f;
  static constexpr int     kJ2J4ArriveDebounce = 20;      // 0.2s @ 100Hz
  static constexpr double  kJ2J4ZeroOffsetRad  = 95.0 * M_PI / 180.0;
  static constexpr double  kJ2J4ArriveThreshRad= 1.0  * M_PI / 180.0;

  // System state management
  void set_system_state(SystemState new_state);
  SystemState get_system_state() const;
  std::string system_state_to_string(SystemState state) const;

  // ✅ Teaching Mode 관련 멤버 함수들
  void teaching_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void set_can_logging_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  void get_can_logging_status_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_manual_origin_j2_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_manual_origin_j4_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void set_manual_origin_j2_j4_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool set_manual_joint_origin(size_t joint_index, std::string & message);
  void apply_origin_in_write(size_t joint_index);   // sleep 없는 write() 전용
  void start_j2j4_stopper_homing();
  void run_j2j4_stopper_homing_step();
  void j2j4_free_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void bed_teach_free_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void start_teaching_mode();
  void stop_teaching_mode();
  bool get_end_effector_pose(double& x, double& y, double& z,
                             double& roll, double& pitch, double& yaw);
  void hall_states_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void manual_origin_command_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  bool is_hall_triggered(size_t joint_index);
  void start_hall_homing();
  bool run_hall_homing_step();
  void finish_joint_homing();
  void advance_to_next_joint();
  void fail_hall_homing(const std::string & reason);


  // 안전 기능
  void emergency_stop_all_motors();
  bool validate_teaching_safety();
};

}  // namespace sfbot_can

#endif  // SFBOT_CAN__RRBOT_HPP_
