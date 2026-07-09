# CLAUDE.md — 6-DOF 로봇암 적층 프로젝트

> Claude Code 세션 연속성을 위한 현황 문서. 새 세션 시작 시 이 파일을 먼저 읽을 것.
> 마지막 업데이트: 2026-07-08

---

## 프로젝트 개요

CubeMars 모터 기반 6축 로봇암으로 재료를 베드 위에 적층하는 시스템.
ROS2 Control + MoveIt2 기반. 주요 스크립트: `scripts/planar_layer_motion.py`.

```
Robot Launch → MoveIt Launch → (베드 교시) → Run Stacking
```

GUI 런처: `~/ros2_ws/src/my_robot_operator_tools/my_robot_operator_tools/launcher_gui.py`

---

## 현재 상태 (2026-07-08)

### 완료된 것
- 베드 4점 교시 완료 (`config/bed_config.yaml` updated_at: 2026-07-08T17:45:36)
- Run Stacking 기본 동작 확인 (`[완료] Planar layer motion 완료.` 출력)
- Approach 이상동작 이슈 해결 완료 (커밋 9a994b1)
- **Circle 패턴 IK 실패 해결** → `tool.force_tcp_x_down: false`로 변경. 교시 자세를 원 전체에 사용하니 Cartesian 100% 계획 성공, r=5.5mm에서 원 완성 확인.
- **원 찌그러짐(선으로 붕괴) 해결** → `trajectory_catchup_time_sec` 1.5→0.4 (xacro). J4 추종오차 59%→9%, 원이 둥글게 나옴. 가능 반경 5.5→8mm 확대. 원인은 백래시가 아니라 느린 catchup(제어 튜닝).

### 현재 미해결 이슈
- **10mm 원은 아직 `-4`** — trial1(10mm)은 접근은 통과하나 원 실행 중 `-4`. 8mm는 성공.
- **시작점 진입부 wobble** — 원 진입점에서 약간 왕복/오버슈트 (Top View 우측).
- 가끔 `JTC timed out` (trial2 접근) — 간헐적.

---

## 해결됨: Circle 패턴 IK 실패 (2026-07-08)

### 원래 증상
`force_tcp_x_down: true`에서 Cartesian 79.5% 실패, seeded IK도 항상 point 58(290°)에서 실패 → 전 반경 ABORT.
원인: `make_tcp_x_down_quat`이 TCP를 강제로 수직 아래로 고정 → 290°(world `[+0.66X,+0.75Y]`, workspace 경계)에서 J5/J6 한계로 IK 해 없음(error=-31).

### 해결책 (적용 완료)
```yaml
tool:
  force_tcp_x_down: false   # 기존 true
```
`planar_layer_motion.py:1557-1558`의 강제 자세 덮어쓰기를 건너뛰고, 교시 quaternion
`[0.666983, -0.334207, -0.58109, -0.325228]`을 원 전체 waypoint에 사용.
→ Cartesian 100% 계획 성공, 모든 점 IK 풀림. **r=5.5mm에서 원 완성 확인.**

### 참고: co-rotation은 실패했었음
`circle_waypoints()`에 `approach_axis=tcp_approach` 추가 → 오히려 전 반경 실패.
현재 코드는 `approach_axis=None`으로 되돌려져 있음 (`scripts/planar_layer_motion.py:1923`).

---

## 해결됨: 원 찌그러짐 = J4 추종 불량 (2026-07-08)

### 원래 증상
원이 둥글지 않고 납작한 선/타원으로 붕괴 (executed png Top View). 관절 추종 플롯에서 J4가 명령 진동 중에도 실제값 평평.

### 진단 (핵심 논리)
- J4 추종오차 = 명령 vs **모터 로터 엔코더** 위치. CubeMars 엔코더는 감속기 앞 로터에 있어 **백래시는 안 잡힘** → 로터 자체가 밀린다는 건 백래시가 아니라 **모터 위치루프가 못 따라가는 것**.
- 원인: `trajectory_catchup_time_sec = 1.5s`가 위치오차를 1.5초에 걸쳐 천천히 메꿈(my_robot.cpp:759-762). 점대점엔 부드럽지만 원 같은 왕복 모션엔 못 쫓아감. J4가 이 자세에서 요구 진폭 최대라 가장 심하게 드러남.
- 부수 요인: SET_POS_SPD 속도 양자화 `int16(output_rpm × 감속비×극쌍)`, AK70-10은 계수 210으로 거침.

### 해결책 (적용 완료)
```xml
<!-- description/ros2_control/ros2_control_my_robot.xacro -->
<param name="trajectory_catchup_time_sec">0.4</param>  <!-- 기존 1.5 -->
```
결과: J4 원구간 오차 59%→9%, 원이 둥글게 나옴. 반경 5.5→8mm 확대.
**주의: 이 xacro는 Robot Launch 시 1회 로드 → 값 바꾸면 Robot Launch 재시작 필수.**
**빌드 주의: `~/cubemars_ros2_control/install`은 symlink 아님(실제파일). 소스만 고치면 stale. 두 install(`~/ros2_ws/install`=symlink OK, `~/cubemars_ros2_control/install`=수동 cp 또는 colcon build) 모두 맞춰야 함. 런처가 cubemars install을 나중에 source하여 우선권 가짐.**

---

## 미해결: 10mm 원 `-4` + 진입부 wobble

### 증상 (catchup 0.4s 상태)
- r=10mm: 시작점 접근은 통과, **원 실행 중** `-4` → 반경 축소
- r=8mm: 성공 (둥근 원)
- 원 진입점에서 약간의 왕복/오버슈트, 간헐적 `JTC timed out`

### 다음에 시도할 것
1. **catchup 더 낮추기** (0.4→0.3) — 10mm 추종 여유 확보 시도 (너무 낮으면 오버슈트/진동 리스크).
2. **velocity_ 상한 상향** (현재 5rpm) — 큰 반경일수록 필요 관절속도 증가.
3. **JTC path tolerance 소폭 완화** (link*_joint trajectory 0.05→0.08) — 8mm 품질 확인 후.
4. 8mm 수용하고 실제 적층 품질/non-planar 확장으로 진행.

---

## 중요 운영 절차

### 베드 교시 (Teach Bed 4-Point) 순서
**반드시 이 순서를 지킬 것. 순서가 틀리면 로봇이 손으로 안 움직임.**

```
1. [Bed Free ON]       → /bed_teach/free_drive true → 6축 duty 0%
2. 손으로 로봇을 각 홈으로 이동
3. [Teach Bed 4-Point] → Enter 4번으로 4점 저장
4. [Bed Free OFF]      → 포지션 홀드 복귀 (반드시!)
```

터미널 직접 실행:
```bash
ros2 topic pub --once /bed_teach/free_drive std_msgs/msg/Bool "{data: true}"
# ... 손으로 이동 후 ...
python3 scripts/teach_bed_four_points.py
ros2 topic pub --once /bed_teach/free_drive std_msgs/msg/Bool "{data: false}"
```

### Run Stacking 실행 전 체크리스트
```
□ Bed Free OFF 상태인지 확인
□ Robot Launch 실행 중
□ MoveIt Launch 실행 중
□ 베드 교시 완료 (bed_config.yaml 확인)
```

---

## 현재 bed_config.yaml 핵심 값

```yaml
bed:
  origin.position: [-0.128318, 0.422802, 0.181685]
  physical_width:  0.066639   # 6.7cm
  physical_height: 0.135812   # 13.6cm
  calibration:
    method: four_point_flat_z
    updated_at: 2026-07-08T17:45:36
    axis_u: [0.932158, -0.362053, 0.0]   # 베드 U축 (world 기준)
    axis_v: [-0.362053, -0.932158, 0.0]  # 베드 V축

layer:
  pattern: circle
  helix_radius_scale: 0.5     # base_radius = min(width,height)*0.5 = 10mm (10/7.5mm은 -4 실패, 5.5mm 성공)
  helix_start_angle_deg: 0.0
  num_layers: 1

motion:
  cartesian_min_fraction: 0.99
  velocity_scale: 0.3
  direct_teach_approach: true
  teach_move_duration: 15.0

teach:
  joint_positions_deg: [15.5, -65.4, 29.9, 43.7, 44.9, 0.0]

tool:
  force_tcp_x_down: false     # ← true였을 때 circle IK 실패. false로 해결 (교시 자세 사용)
```

---

## 코드 구조

| 파일 | 역할 |
|------|------|
| `scripts/planar_layer_motion.py` | 메인 적층 스크립트 |
| `scripts/teach_bed_four_points.py` | 베드 4점 교시 |
| `scripts/teach_bed_by_joints.py` | 조인트 기반 베드 교시 |
| `hardware/my_robot.cpp` | 하드웨어 인터페이스 (CAN, 교시모드, 호밍) |
| `config/bed_config.yaml` | 베드/레이어/모션 파라미터 |
| `SYSTEM_TECHNICAL_OVERVIEW.md` | 전체 시스템 기술 문서 |

### 주요 ROS2 토픽/서비스
| 토픽 | 용도 |
|------|------|
| `/bed_teach/free_drive` (Bool) | 전축 duty 0% free-drive ON/OFF |
| `/teaching_mode` (Bool) | brake current 기반 교시 모드 |
| `/joint_states` | 현재 조인트 상태 |
| `/hall_states` | 홀센서 상태 (호밍용) |

---

## 알려진 하드웨어 특성

- **Bed Free OFF 잊으면** 포지션 홀드 없이 암이 흘러내릴 수 있음
- `measured_z_range_m: 0.034257` (34mm) — 4점 교시 시 Z 편차가 크게 나옴. `flat_average`로 강제 평탄화 중
- `front_back_width_error_m: 0.044063` (44mm) — 베드가 직사각형이 아님 (사다리꼴)
