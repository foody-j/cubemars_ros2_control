# 6-DOF 로봇암 ROS2 Control 시스템 기술 문서

> 계획서 작성용 기술 상세 참고 자료  
> 작성 기준: 현재 구현 완료된 코드 기반 (2026년 4월)

---

## 1. 전체 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                     사용자 / 상위 시스템                        │
└──────────────────────────┬──────────────────────────────────┘
                           │ ROS2 Topic / Action
┌──────────────────────────▼──────────────────────────────────┐
│                    MoveIt2 (move_group)                      │
│  - OMPL Planner (ompl_interface/OMPLPlanner)                │
│  - AddTimeOptimalParameterization (시간 파라미터화)            │
│  - FollowJointTrajectory Action Client                       │
└──────────────────────────┬──────────────────────────────────┘
                           │ JointTrajectory
┌──────────────────────────▼──────────────────────────────────┐
│              ROS2 Control (controller_manager)               │
│  - joint_trajectory_controller (JTC)                        │
│  - joint_state_broadcaster                                  │
└──────────────────────────┬──────────────────────────────────┘
                           │ position command/state interface
┌──────────────────────────▼──────────────────────────────────┐
│      MyRobotSystemHardware (hardware_interface::SystemInterface) │
│  - 시스템 상태 머신 (IDLE/HOMING/READY/RUNNING/ERROR)         │
│  - Hall 센서 호밍 FSM                                         │
│  - 조인트 리밋 가드                                            │
└──────────────────────────┬──────────────────────────────────┘
                           │ SocketCAN (Linux kernel)
┌──────────────────────────▼──────────────────────────────────┐
│                   CanComms (CAN 통신 드라이버)                  │
│  - USB-CAN 어댑터 (can0, can1 ... 자동 탐색)                   │
│  - Bitrate: 1 Mbps                                          │
│  - 모터별 CAN ID: 0x601~0x606                                │
│  - 별도 command_thread (비동기 전송)                           │
└──────────────────────────┬──────────────────────────────────┘
                           │ CAN Bus
┌──────────────────────────▼──────────────────────────────────┐
│           CubeMars 액추에이터 (6개)                             │
│  J1/J2: AK60-84  (감속비 64:1, 최대 토크 48Nm)                │
│  J3/J4: AK70-10  (감속비 10:1, 최대 토크 8.3Nm)               │
│  J5/J6: AK60-6   (감속비 6:1,  최대 토크 3Nm)                 │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 하드웨어 구성

### 2-1. 로봇암 본체

| 항목 | 내용 |
|------|------|
| 자유도 | 6-DOF |
| 설계 도구 | Fusion360 |
| 제작 방식 | FDM 3D 프린팅 (PLA / 일부 금속 파트) |
| 엔드이펙터 | 식품 노즐 (현재) / 그리퍼 전환 예정 |
| 로봇 베이스 | `base_link` → `world` 고정 조인트 |
| TCP 링크 | `tcp` (`tool_nozzle` → `tcp` 고정 조인트) |

### 2-2. 액추에이터 (CubeMars)

| 조인트 | 모터 모델 | 감속비 | 최대 RPM | 최대 토크 | Pole Pairs | 역할 |
|--------|-----------|--------|----------|----------|------------|------|
| J1 | AK60-84 | 64:1 | 23 RPM | 48 Nm | 21 | 베이스 회전 (Z축) |
| J2 | AK60-84 | 64:1 | 23 RPM | 48 Nm | 21 | 숄더 피치 (X축) |
| J3 | AK70-10 | 10:1 | 148 RPM | 8.3 Nm | 21 | 엘보 회전 (Z축) |
| J4 | AK70-10 | 10:1 | 148 RPM | 8.3 Nm | 21 | 손목 피치 (X축) |
| J5 | AK60-6 | 6:1 | 420 RPM | 3 Nm | 14 | 손목 회전 (Z축) |
| J6 | AK60-6 | 6:1 | 420 RPM | 3 Nm | 14 | 엔드이펙터 회전 |

### 2-3. Hall 센서

- 조인트별 홀센서 1개 (J1~J5, J6은 그리퍼로 스킵)
- ROS2 토픽 `/hall_states` (`std_msgs/Int32MultiArray`) 수신
- 아두이노 기반 Hall 센서 인터페이스 (`hall_sensor.ino`)
- 트리거 상태값: `1` (설정 가능, `hall_trigger_state` 파라미터)

### 2-4. CAN 통신 인터페이스

| 항목 | 내용 |
|------|------|
| 인터페이스 | USB-CAN 어댑터 (멀티허브 연결) |
| 리눅스 드라이버 | SocketCAN (`linux/can.h`, `linux/can/raw.h`) |
| Bitrate | 1,000,000 bps (1 Mbps) |
| 인터페이스 이름 | `can0`, `can1` ... 자동 탐색 |
| 모터 CAN ID | 0x601 ~ 0x606 (J1~J6) |
| 최대 연결 재시도 | 3회 (점진적 딜레이: 500ms × retry) |

---

## 3. CAN 통신 프로토콜 (CubeMars)

### 3-1. 제어 모드

| 모드 | CAN ID | 설명 |
|------|--------|------|
| Velocity Mode | 0x6xx | 속도 명령 (rad/s → ERPM 변환) |
| Position-Velocity Mode | 0x6xx | 위치 + 속도 + 가속도 동시 명령 |

### 3-2. 프로토콜 변환

```
위치 (도 → 프로토콜):   int32 = position_deg × 10000
속도 (ERPM → Velocity 모드):  int32 = ERPM 직접 전송
속도 (ERPM → Position-Vel 모드): int16 = ERPM / 10  (범위: ±32767)
가속도 (ERPM/s² → 프로토콜):  int16 = ERPM_per_s² / 10

ERPM 변환:
  ERPM = output_RPM × reduction_ratio × pole_pairs
  예) J1 (AK60-84): ERPM = output_RPM × 64 × 21 = output_RPM × 1344
```

### 3-3. last_sent 타이밍 버그 수정 이력

```cpp
// [수정 전] write_velocity()가 전송 직후 쿨다운으로 다음 전송 막힘
motor_commands_[id].last_sent = std::chrono::steady_clock::now();

// [수정 후] -20ms로 설정해 다음 CAN 전송이 즉시 가능하도록
motor_commands_[id].last_sent = std::chrono::steady_clock::now()
                                - std::chrono::milliseconds(20);
```

---

## 4. ROS2 Control 하드웨어 인터페이스

### 4-1. 플러그인 정보

```
클래스명: MyRobotSystemHardware
베이스:   hardware_interface::SystemInterface
플러그인: my_robot_ros2_control/MyRobotSystemHardware
```

### 4-2. 라이프사이클 콜백

| 콜백 | 동작 |
|------|------|
| `on_init()` | URDF 파라미터 파싱, 조인트 리밋 로드 |
| `on_configure()` | CAN 연결, ROS2 노드 생성, Hall 센서 구독 |
| `on_activate()` | Hall 호밍 시작 (또는 호밍 스킵), HOMING 상태 진입 |
| `on_deactivate()` | 모든 모터 속도 0 명령, 정지 |
| `read()` | CAN에서 모터 위치 읽기 → `pos_[]` 업데이트 |
| `write()` | 상태 머신에 따라 CAN 명령 전송 |

### 4-3. 조인트 인터페이스

```
Command Interface: position (rad)
State Interface:   position (rad)
```

### 4-4. 조인트 리밋 (ros2_control.xacro 정의)

| 조인트 | min (rad) | max (rad) | min (도) | max (도) |
|--------|-----------|-----------|----------|----------|
| J1 (`link1_1_joint`) | -11π/9 | π/2 | -220° | +90° |
| J2 (`link2_1_joint`) | -π/2 | π/2 | -90° | +90° |
| J3 (`link3_1_joint`) | -π/9 | 3π/2 | -20° | +270° |
| J4 (`link4_1_joint`) | -19π/36 | 19π/36 | -95° | +95° |
| J5 (`link5_1_joint`) | -π/9 | 31π/18 | -20° | +310° |
| J6 (`link6_1_joint`) | -19π/36 | 19π/36 | -95° | +95° |

---

## 5. 시스템 상태 머신

```
            on_activate()
  IDLE ─────────────────► HOMING
                             │
                    Hall 호밍 완료
                             │
                             ▼
                 ┌─────── READY ◄──────────────┐
                 │  (명령값≈현재값, debounce 50)  │
                 │                              │
         명령값≠현재값                      모션 완료
         (>0.01 rad)                    (debounce 50 cycles)
                 │                              │
                 └────────► RUNNING ────────────┘

   어느 상태에서든 에러 발생 → ERROR
```

### 상태별 동작

| 상태 | write() 동작 |
|------|-------------|
| IDLE | CAN 명령 없음 |
| HOMING | Hall 호밍 FSM 실행 (속도 명령) |
| READY | 현재 위치 유지 (position-velocity 명령) |
| RUNNING | 궤적 명령 추종 (position-velocity 명령) |
| ERROR | 모든 모터 속도 0 |

### READY ↔ RUNNING 전환 조건

```cpp
POSITION_THRESHOLD = 0.01 rad (~0.57도)

READY → RUNNING: 임의 조인트에서 |cmd - pos| > 0.01 rad
RUNNING → READY: 모든 조인트 |cmd - pos| ≤ 0.01 rad 이 50 사이클 연속 유지
                 (debounce로 빠른 토글 방지)
```

---

## 6. Hall 센서 기반 호밍 FSM

### 6-1. 단계별 동작

```
J1 시작
  │
  ▼
[APPROACH]
  모터를 hall_homing_velocity (1.0 rad/s) 로 방향 이동
  홀센서 트리거 감지까지 계속
  │ 트리거 감지
  ▼
[BACKOFF]
  반대 방향으로 hall_backoff_duration_sec (0.4s) 동안 이동
  (센서 엣지에서 충분히 벗어나기)
  │ 0.4초 경과
  ▼
[REAPPROACH]
  다시 hall_reapproach_velocity (1.0 rad/s) 로 천천히 접근
  홀센서 트리거 재감지 → 정밀 원점 포착
  │ 트리거 감지
  ▼
[MOVING_TO_HOME]  ← 오프셋이 있는 경우만
  트리거 위치에서 hall_home_offsets_deg 만큼 이동
  목표 위치 0.5도 이내 도달 확인
  │ 완료
  ▼
해당 조인트 원점(0 rad) 설정 → 다음 조인트로
```

### 6-2. 호밍 파라미터

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `hall_homing_joints_count` | 5 | 호밍할 조인트 수 (J6 스킵) |
| `hall_homing_velocity` | 1.0 rad/s | APPROACH 속도 |
| `hall_reapproach_velocity` | 1.0 rad/s | REAPPROACH 속도 |
| `hall_backoff_duration_sec` | 0.4 s | BACKOFF 지속 시간 |
| `hall_joint_timeout_sec` | 60.0 s | 조인트별 호밍 타임아웃 |
| `hall_trigger_state` | 1 | 트리거 감지 상태값 |

### 6-3. 호밍 방향 및 오프셋

| 조인트 | 방향 | 오프셋 (도) | 설명 |
|--------|------|------------|------|
| J1 | +1 | 0 | 트리거 위치 = 0도 |
| J2 | +1 | -100 | 트리거에서 -100도 이동 |
| J3 | +1 | -202.5 | 트리거에서 -202.5도 이동 |
| J4 | +1 | -95 | 트리거에서 -95도 이동 |
| J5 | -1 | 0 | 역방향, 트리거 위치 = 0도 |
| J6 | +1 | 0 | 스킵 (그리퍼) |

### 6-4. 오프셋 적용 공식

```
target_deg = trigger_position_deg + offset_deg
joint_home_offset_rad = -trigger_position_rad
pos[j] = 0, cmd[j] = 0  (원점 재정의)
```

### 6-5. 모니터링

```
5초마다 로그 출력:
  🔍 Homing J{n} [PHASE] hall={0/1} pos={현재각도}°
```

---

## 7. MoveIt2 연동

### 7-1. 구성

| 항목 | 내용 |
|------|------|
| Planning Group | `arm` |
| Base Link | `base_link` |
| Tip Link (TCP) | `tcp` |
| Planner | OMPL (`ompl_interface/OMPLPlanner`) |
| Time Parameterization | `AddTimeOptimalParameterization` |
| Controller | `joint_trajectory_controller` (JTC) |

### 7-2. MoveIt 핵심 설정

```python
# planning_pipeline (Humble 네임스페이스 필수)
{
  "move_group": {
    "planning_plugin": "ompl_interface/OMPLPlanner",
    "request_adapters": [
      "AddTimeOptimalParameterization",   # timestamp 자동 생성
      "FixWorkspaceBounds",
      "FixStartStateBounds",
      "FixStartStateCollision",
      "FixStartStatePathConstraints",
    ]
  }
}

# trajectory_execution
{
  "moveit_manage_controllers": False,      # JTC를 직접 관리하지 않음
  "allowed_execution_duration_scaling": 2.0,
  "allowed_start_tolerance": 0.5,
  "execution_duration_monitoring": False,
}
```

> **주의 (Humble 이슈):** `planning_plugin` 파라미터는 반드시  
> `move_group.planning_plugin` 네임스페이스로 설정해야 적용됨.  
> dict key를 `"move_group"`으로 감싸는 방식으로 해결.

### 7-3. MoveIt 속도/가속도 리밋 (joint_limits.yaml)

| 조인트 | max_velocity (rad/s) | max_acceleration (rad/s²) |
|--------|---------------------|--------------------------|
| J1 | 0.72 | 0.9 |
| J2 | 0.72 | 0.9 |
| J3 | 2.7 | 1.8 |
| J4 | 2.7 | 1.8 |
| J5 | 3.6 | 2.7 |
| J6 | 3.6 | 2.7 |

> 원래 값의 1.8배로 증가 적용 (2026년 4월 기준)

### 7-4. 역기구학 (Kinematics)

```yaml
# kinematics.yaml
arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
```

---

## 8. 소프트웨어 패키지 구조

```
ros2_ws/src/my_robot_ros2_control/
├── hardware/
│   ├── my_robot.cpp                    ← 메인 하드웨어 인터페이스 (951줄)
│   └── include/my_robot_ros2_control/
│       ├── my_robot.hpp                ← 클래스 선언, SystemState enum
│       ├── motor_can_driver.hpp        ← CAN 통신 드라이버 (CanComms 클래스)
│       ├── motor_data.hpp              ← 모터 데이터 구조체
│       ├── cubemars_motors.hpp         ← 모터 사양, 프로토콜 변환 함수
│       ├── teaching_data_logger.hpp    ← 교시 데이터 로깅
│       └── can_csv_logger.hpp          ← CAN 데이터 CSV 기록
│
├── description/
│   ├── urdf/
│   │   └── my_robot.urdf.xacro         ← 메인 URDF
│   ├── ros2_control/
│   │   └── ros2_control_my_robot.xacro ← ros2_control 파라미터
│   └── meshes/                         ← STL 메시 파일들
│
├── config/
│   ├── moveit/
│   │   ├── my_robot.srdf               ← MoveIt 시맨틱 설명 (그룹, 충돌)
│   │   ├── kinematics.yaml             ← KDL IK 설정
│   │   ├── joint_limits.yaml           ← MoveIt 속도/가속도 리밋
│   │   ├── moveit_controllers.yaml     ← JTC 연동 설정
│   │   └── moveit.rviz                 ← RViz 설정
│   ├── bed_config.yaml                 ← 베드 교시 + 적층 파라미터
│   └── safe_poses.yaml                 ← 안전 자세 정의
│
├── scripts/
│   ├── teach_bed.py                    ← 베드 원점 교시 (TF → yaml 저장)
│   ├── planar_layer_motion.py          ← 평면 적층 모션 실행
│   ├── cartesian_test.py               ← Cartesian 경로 단순 테스트
│   └── verify_hall_polarity.py         ← Hall 센서 극성 확인
│
└── bringup/launch/
    └── (my_robot.launch.py는 cubemars_ros2_control 패키지에 있음)

cubemars_ros2_control/
├── bringup/launch/
│   ├── my_robot.launch.py              ← 메인 런치 (RSP + controller_manager + RViz)
│   └── moveit.launch.py                ← MoveIt move_group 런치
└── bringup/config/
    └── my_robot_controllers.yaml       ← JTC 컨트롤러 설정
```

---

## 9. 런치 시스템

### 9-1. 실행 명령어

```bash
# 터미널 1: 로봇 하드웨어 + 컨트롤러
ros2 launch my_robot_ros2_control my_robot.launch.py

# 터미널 2: MoveIt (호밍 완료 후)
ros2 launch my_robot_ros2_control moveit.launch.py

# 옵션: 호밍 스킵 (개발/테스트용)
ros2 launch my_robot_ros2_control my_robot.launch.py use_hall_homing:=false
```

### 9-2. 런치 플로우

```
my_robot.launch.py 실행
  ├── robot_state_publisher  (URDF → /robot_description, TF)
  ├── controller_manager     (ros2_control 로드)
  │     ├── joint_state_broadcaster 활성화
  │     └── joint_trajectory_controller 활성화
  └── rviz2 (moveit.rviz 설정)

  → hardware on_activate() 호출
  → Hall 호밍 자동 시작 (IDLE → HOMING)
  → 호밍 완료 시 READY 상태

moveit.launch.py 실행 (별도)
  └── move_group 노드
        ├── OMPL 플래너 로드
        ├── kinematics (KDL)
        ├── joint_limits 로드
        └── JTC에 trajectory 전달
```

---

## 10. 평면 적층 모션 시스템

### 10-1. 개요

로봇 TCP를 한 점에 교시하면 그 위치를 기준으로 지정한 패턴(래스터/원형/나선)을 레이어별로 자동 실행하는 시스템.

### 10-2. 좌표계 정의

```
교시한 TCP orientation → 베드 좌표계 정의

  TCP local X → 베드 가로 방향 (width)
  TCP local Y → 베드 세로 방향 (height)
  TCP local Z → 베드 법선 (표면에서 위 방향)

베드 프레임 좌표 → 로봇 월드 좌표 변환:
  world_pos = bed_origin + R @ [u, v, w]
  (R: 쿼터니언 → 회전행렬)

레이어 오프셋:
  layer_origin = bed_origin + layer_idx × layer_height × bed_Z
```

### 10-3. 래스터 패턴 생성

```python
# 짝수 라인: 왼쪽→오른쪽, 홀수 라인: 오른쪽→왼쪽
v = 0.0
while v <= height:
    u_start, u_end = (0, width) if even else (width, 0)
    waypoints.append([u_start, v, 0])  # 라인 시작
    waypoints.append([u_end,   v, 0])  # 라인 끝
    v += line_spacing
```

### 10-4. 실행 파이프라인

```
bed_config.yaml 로드
  │
  ▼
approach 위치로 joint-space 이동
  │  (set_goal_state + arm.plan())
  ▼
For each layer:
  ├── 레이어 표면으로 Cartesian 하강 (1 waypoint)
  ├── 래스터 waypoint 리스트 생성
  ├── compute_cartesian_path 실행
  └── 다음 레이어 전 approach_height로 리프트
  │
  ▼
완료 후 안전 높이로 복귀
```

### 10-5. bed_config.yaml 파라미터

```yaml
bed:
  origin:
    position: [x, y, z]           # teach_bed.py가 저장
    orientation: [qx, qy, qz, qw] # teach_bed.py가 저장
  width: 0.05                     # 베드 가로 (m)
  height: 0.05                    # 베드 세로 (m)
  stacking: world_z               # 레이어 방향

layer:
  pattern: circle                 # raster / circle / helix
  num_layers: 3
  layer_height: 0.005             # 레이어 간격 (5mm)
  line_spacing: 0.010             # 래스터 라인 간격 (10mm)

motion:
  z_offset: 0.02                  # 교시 위치 추가 Z 오프셋
  approach_height: 0.01           # 베드 위 접근 높이
  cartesian_max_step: 0.003       # Cartesian 보간 간격 (3mm)
  velocity_scale: 0.30            # 속도 스케일 (0~1)
```

---

## 11. 핵심 해결 이슈 (개발 중 발견된 기술적 문제)

| 이슈 | 원인 | 해결 방법 |
|------|------|----------|
| APPROACH 중 모터 멈칫거림 | `write_velocity()`의 `last_sent=now()` → 다음 전송 막힘 | `last_sent = now() - 20ms`로 수정 |
| READY↔RUNNING 빠른 토글 | `POSITION_THRESHOLD=0.001 rad` 너무 작음 | 0.01 rad로 증가 + 50 사이클 debounce |
| MoveIt Goal 거부 | `moveit_manage_controllers=True` → "continuation" 모드 전송 | `False`로 변경 |
| CHOMP timestamp=0 | CHOMP는 time parameterization 미적용 | OMPL로 전환 |
| OMPL planning_plugin 무시 | Humble에서 파이프라인 네임스페이스 필요 | `{"move_group": {"planning_plugin": ...}}` |
| xacro 변경 미적용 | colcon이 xacro를 install/에 복사 | `--symlink-install` 사용 |
| USB-CAN 연결 끊김 | 멀티허브 전원/신호 불안정 | 뽑았다 재연결로 해결 (하드웨어 이슈) |

---

## 12. 향후 확장 계획

### 12-1. TCP 홀센서 베드 호밍 (자동 좌표계 캘리브레이션)

```
베드에 네오디뮴 자석 고정
TCP에 홀센서 부착
  │
  ▼
로봇이 베드 표면을 XY 스캔하며 자석 위치 탐색
  │
  ▼
자석 위치 = 베드 원점
→ bed_config.yaml의 origin 자동 업데이트
→ teach_bed.py 불필요
```

### 12-2. Pick & Place 시스템 (계획서 목표)

```
에지 AI (NVIDIA Jetson) → 품질 판정 결과
  │ MQTT Publish
  ▼
로봇 제어기 (MQTT Subscribe)
  │
  ▼
Hand-Eye Calibration → 카메라 좌표 → 로봇 좌표 변환
  │
  ▼
그리퍼 파지 → 분류 (양품/불량품 라인)
```

### 12-3. MQTT 통신 구조 (설계 예정)

```
Topic: /food_quality/result  
Payload: {"id": 1, "label": "good/bad", "confidence": 0.95, "bbox": [...]}

Publisher: 에지 AI (Jetson)
Subscriber: 로봇 제어 노드
```

---

## 13. 개발 환경

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS2 Humble |
| MoveIt | MoveIt2 (Humble) |
| 빌드 | `colcon build --symlink-install` |
| 언어 | C++17 (하드웨어 드라이버), Python3 (스크립트) |
| IDE | (Linux 네이티브 터미널 기반) |
| 3D 설계 | Fusion360 |
| 데이터 라벨링 | Roboflow (예정) |
| 에지 디바이스 | NVIDIA Jetson 시리즈 (예정) |
| AI 프레임워크 | YOLO 계열 + TensorRT (예정) |
