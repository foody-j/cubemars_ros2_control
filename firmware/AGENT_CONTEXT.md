# Agent Context

이 문서는 에이전트 간 작업 인수인계용이다.

작업 시작 전에 이 파일과 관련 코드의 최신 상태를 먼저 읽는다. 추측으로 빌드, 업로드, 로봇 실행, 모터 구동을 하지 않는다.

## 운영 규칙

- 작업 시작 전에 이 문서를 먼저 읽는다.
- 사용자가 명시적으로 요청하기 전까지 빌드하지 않는다.
- 사용자가 명시적으로 요청하기 전까지 아두이노에 업로드하지 않는다.
- 사용자가 명시적으로 요청하기 전까지 모터 구동 명령을 실제로 보내지 않는다.
- 사용자가 명시적으로 요청하기 전까지 로봇 모션을 실행하지 않는다.
- 변경 후 중요한 결정, TODO, 주의사항은 이 문서에 업데이트한다.
- 상세 설계와 사용법은 `EXTRUDER_SERIAL_NOTES.md`에 둔다.
- 이 파일은 현재 상태, 결정사항, 다음 작업, 금지사항 중심으로 짧게 유지한다.

## 현재 목표

ROS2 기반 로봇팔 적층 작업에 아두이노 압출 스텝모터를 통합한다.

최종 흐름은 다음과 같다.

```text
로봇팔 Approach 위치 이동
  -> 압출 A 모터 시작
  -> 사용자가 노즐 끝 소재 도착 확인
  -> Enter 입력
  -> 적층 모션 시작
  -> 적층 완료/실패/중단 시 압출 A 모터 정지
```

## 현재 결정사항

- 아두이노는 YAML을 직접 읽지 않는다.
- ROS2 노드가 YAML을 읽고 시리얼 명령을 보낸다.
- 압출기 제어는 `/extruder/command` 토픽으로 한다.
- 압출기 상태는 `/extruder/status` 토픽으로 받는다.
- 노즐 끝에 소재가 도착하는 시점은 사람이 직접 보고 Enter로 확정한다.
- `planar_layer_motion.py`의 적층 시작 Enter 전에 A 모터를 prime한다.
- 직접 시리얼 제어보다 ROS2 토픽을 통해 `extruder_serial_node`에 명령을 보내는 구조를 우선한다.

## 제어 모드

| 모드 | 의미 |
| --- | --- |
| `MANUAL` | 스위치 기반 테스트. 시작/정지/방향은 스위치가 담당한다. |
| `MIXED` | 스위치와 ROS2 명령을 모두 허용한다. |
| `ROS` | 스위치 입력을 무시하고 ROS2 명령만 허용한다. |

YAML에서는 소문자로 설정한다.

```yaml
control_mode: "mixed"
```

시리얼 명령은 대문자를 사용한다.

```text
MODE MIXED
MODE MANUAL
MODE ROS
```

## 관련 파일

- 압출기 상세 설계/사용법: `/home/yjglaptop/et_motor/EXTRUDER_SERIAL_NOTES.md`
- 아두이노 펌웨어: `/home/yjglaptop/et_motor/extruder_no_oled/extruder_no_oled.ino`
- 로컬 YAML 예시: `/home/yjglaptop/et_motor/config/extruder_params.yaml`
- ROS2 패키지: `/home/yjglaptop/cubemars_ros2_control`
- ROS2 압출기 YAML: `/home/yjglaptop/cubemars_ros2_control/config/extruder_params.yaml`
- ROS2 시리얼 노드: `/home/yjglaptop/cubemars_ros2_control/src/extruder_serial_node.cpp`
- ROS2 압출기 런치: `/home/yjglaptop/cubemars_ros2_control/bringup/launch/extruder.launch.py`
- ROS2 압출기 GUI: `/home/yjglaptop/ros2_ws/src/my_robot_operator_tools/my_robot_operator_tools/extruder_gui.py`
- 적층 모션 스크립트: `/home/yjglaptop/cubemars_ros2_control/scripts/planar_layer_motion.py`
- 4점 베드 교시 스크립트: `/home/yjglaptop/cubemars_ros2_control/scripts/teach_bed_four_points.py`

## 현재 구현 상태

- 2026-06-18 기준 압출기 단독 제어는 구현 및 실제 모터 구동 확인 완료.
- 기존 `oled_rgb.ino`는 OLED/RGB 포함 원본으로 남아 있다.
- `extruder_no_oled.ino`는 OLED/RGB 제거 버전이다.
- `extruder_no_oled.ino`에는 스위치 제어와 시리얼 제어가 같이 들어 있다.
- `extruder_no_oled.ino`에는 `MIXED`, `MANUAL`, `ROS` 제어 모드가 들어 있다.
- ROS2 쪽에는 `extruder_serial_node.cpp`, `extruder_params.yaml`, `extruder.launch.py` 초안이 추가되어 있다.
- Operator tools 쪽에는 `/extruder/command`, `/extruder/status` 기반 `extruder_gui`가 추가되어 있다.
- Robot Launcher에는 `Extruder GUI` 버튼이 추가되어 있다.
- Robot Launcher Live State에는 압출기 bridge 상태, 마지막 `/extruder/status`, `Extruder STATUS`, `Extruder STOP ALL`이 추가되어 있다.
- Robot Launcher의 큰 STOP 버튼은 trajectory cancel 전에 압출기 `STOP A/B/C`도 전송한다.
- Robot Launcher는 `Robot System Dashboard` 제목, 상단 상태 카드, Live Telemetry 패널 형태로 1차 UI 개선되어 있다.
- 바탕화면에는 `/home/yjglaptop/Desktop/Extruder_GUI.desktop`가 추가되어 있다.
- `/home/yjglaptop/start_extruder_gui.sh`는 `extruder_serial_node`가 없으면 먼저 `ros2 launch my_robot_ros2_control extruder.launch.py`를 백그라운드로 띄운 뒤 GUI를 실행한다.
- `planar_layer_motion.py`에는 수동 prime 방식의 압출기 1차 연동이 추가되어 있다.
- `bed_config.yaml`에는 `extruder.enabled`, `motor`, `interval_us`, `direction`, `prime.manual_confirm`, `stop_on_finish` 설정이 추가되어 있다.
- 4점 베드 교시가 추가되어 있다. `teach_bed_four_points.py`는 좌앞/우앞/좌뒤/우뒤 홈에서 `base_link -> tcp` 좌표를 읽고 `bed.calibration`에 축/normal/실측 크기를 저장한다.
- `planar_layer_motion.py`는 `bed.calibration.enabled: true`이면 4점 교시 좌표계로 출력 중심, U/V축, 레이어 normal, z offset, approach/lift를 계산한다. calibration이 없으면 기존 1점/월드축 방식으로 fallback한다.
- 현재 감지된 Arduino 포트는 CH340 계열 `/dev/ttyUSB0`이며, ROS2/로컬 YAML 기본 포트도 `/dev/ttyUSB0`로 맞췄다.
- `EXTRUDER_SERIAL_NOTES.md`는 한글 설계/사용법 문서다.
- 빌드 산출물 `build/`, `install/`, `log/`는 삭제했다.

## 다음 작업 후보

- 역할 분리 원칙을 유지한다: Robot Launcher는 실행/상태/긴급정지, `planar_layer_motion.py`는 실제 로봇팔 모션-압출 타이밍, Extruder GUI는 수동 테스트/보정.
- 시스템 구성 원칙: Robot Launcher를 상위 시스템 대시보드로 보고, 세부 조작은 Extruder GUI/Logging GUI/Joint Jog 같은 전용 패널로 유지한다.
- 로봇 런처 압출기 상태/정지 운영 보조 기능은 1차 구현 완료. 추후 UI polish와 상태 판정 강화만 남았다.
- `planar_layer_motion.py` 압출 prime 1차 연동은 구현 완료. 실제 로봇+압출 통합 테스트가 남았다.
- 현재 흐름: approach 완료 후 `MODE/INTERVAL_US/DIR/START C`, 사용자가 노즐 토출 확인 후 Enter, 적층 완료 시 `STOP C`.
- 실제 압출기 연결은 현재 C 모터 채널 기준이다. `bed_config.yaml`의 `extruder.motor: C`를 사용한다.
- 다음 테스트 전제: Robot Launch, MoveIt Launch, `extruder_serial_node`가 살아 있어야 한다.
- 출력 테스트 전 4점 홈을 쓰려면 Robot Launch 재시작 후 런처에서 `Bed Free ON`을 누르고 `Teach Bed 4-Point`를 실행한다. `Teaching ON`은 사용하지 않는다. 순서는 좌앞, 우앞, 좌뒤, 우뒤이며 손으로 노즐 끝을 홈에 살짝 넣은 상태에서 Enter를 누른다. 저장 후 `Bed Free OFF`를 눌러 현재 joint position으로 명령을 동기화한다.
- 4점 베드 손교시용 free-drive는 `/bed_teach/free_drive` (`std_msgs/Bool`) 토픽이다. ON이면 6축 duty 0%를 계속 보내고 position command를 스킵한다. 하드웨어 플러그인 변경이므로 기존 Robot Launch를 재시작해야 토픽이 생긴다. Free 상태에서는 중력으로 팔이 움직일 수 있으니 팔/노즐을 지지하고 비상정지를 가까이 둔다.
- 다음 테스트는 `bed_config.yaml`의 `extruder.interval_us`를 보수적으로 `3000` 정도로 시작하고, 안정 확인 후 낮춘다. 현재 설정은 `1000`.
- 프린팅 파라미터는 추후 YAML로 분리한다: `extruder_interval_us`, `prime_motor`, `prime_mode`, `stop_on_finish` 등.
- 장기적으로 문자열 토픽 명령을 `SetBool`/서비스/액션 또는 job runner 단계로 감싸서 실패 처리와 상태 확인을 명확히 한다.
- 최종적으로는 Robot Launch, MoveIt, extruder ready, approach, prime, operator confirm, layer path, stop extruder, park/finish를 하나의 job runner 상태 머신으로 묶는다.
- 시스템 대시보드는 새 앱을 또 만들기보다 현재 Robot Launcher를 점진적으로 확장한다. 추후 이름/구성은 `Robot System Dashboard` 성격으로 정리한다.
- 아두이노 연결 후 `/dev/ttyUSB*`, `/dev/ttyACM*` 포트를 확인한다.
- 런처 GUI 또는 `ros2 run my_robot_operator_tools extruder_gui`로 압출기 GUI를 연다.

## 현재 주의사항

- 2026-06-18 확인 시점에는 Arduino가 `/dev/ttyUSB0`로 잡혔다.
- 포트가 바뀌면 `/home/yjglaptop/cubemars_ros2_control/config/extruder_params.yaml`의 `port`를 수정해야 한다.
- ENA 극성은 드라이버마다 다르다. 현재 펌웨어는 `ENA HIGH`로 설정한다.
- 드라이버가 `ENA LOW = 활성` 타입이면 펌웨어에서 ENA 출력을 LOW로 바꿔야 한다.
- 비상정지는 별도 하드웨어 차단 스위치가 필요하다.
- 고속 펄스가 더 필요하면 타이머 인터럽트 기반 펄스 생성으로 바꾸는 것이 좋다.

## 최근 변경

- 2026-04-10: OLED/RGB 제거 펌웨어 `extruder_no_oled.ino` 생성.
- 2026-04-10: 시리얼 명령 기반 압출기 제어 프로토콜 추가.
- 2026-04-10: `MIXED`, `MANUAL`, `ROS` 제어 모드 설계 및 펌웨어 반영.
- 2026-04-10: ROS2 통합 초안 파일 추가.
- 2026-04-10: 한글 상세 문서 `EXTRUDER_SERIAL_NOTES.md` 작성.
- 2026-04-10: 노즐 끝 소재 도착 확인은 사람이 보고 Enter를 누르는 방식으로 결정.
- 2026-06-18: Operator tools에 압출기 GUI 추가, 런처 버튼 추가, 기본 포트 `/dev/ttyUSB0`로 갱신.
- 2026-06-18: 바탕화면 `Extruder_GUI.desktop`와 `start_extruder_gui.sh` 추가. 스크립트가 serial bridge를 자동 실행하도록 보강.
- 2026-06-18: GUI에서 실제 모터 구동 확인 완료. 압출기 단독 구현 단계는 완료로 판단.
- 2026-06-18: 추후 통합 방향 결정. 런처는 운영 콘솔, 적층 스크립트는 모션-압출 타이밍, Extruder GUI는 수동 테스트/보정 역할로 분리한다.
- 2026-06-18: Robot Launcher Live State에 압출기 상태/STATUS/STOP ALL을 추가하고, 큰 STOP이 압출기 STOP A/B/C도 보내도록 수정.
- 2026-06-18: `planar_layer_motion.py`에 수동 prime 방식의 압출기 1차 연동 추가. 실제 실행 테스트는 아직 하지 않음.
- 2026-06-18: 실제 압출기 연결이 C 채널임을 확인하여 `bed_config.yaml`의 `extruder.motor`를 `C`로 변경.
- 2026-06-18: 시스템 대시보드는 새 앱 추가보다 Robot Launcher 확장으로 진행하기로 정리. 다음 실제 테스트는 `interval_us`를 3000부터 시작 권장.
- 2026-06-18: Robot Launcher UI를 상위 시스템 대시보드 느낌으로 1차 개선. 상단 Robot/MoveIt/Extruder/Logging 상태 카드와 Live Telemetry 패널 적용.
- 2026-06-18: 4점 베드 교시 구현 추가. `Teach Bed 4-Point` 버튼과 `teach_bed_four_points.py`를 추가했고, `planar_layer_motion.py`가 `bed.calibration`의 U/V축과 normal을 사용하도록 연결.
- 2026-06-19: 4점 베드 손교시용 `Bed Free ON/OFF` 추가. `/bed_teach/free_drive`가 6축 duty 0% free-drive를 제어하며, OFF 시 현재 joint position으로 command를 동기화한다. `my_robot_ros2_control`, `my_robot_operator_tools` 빌드 확인 완료.
- 2026-06-19: 원 경로 흔들림 원인 분석 기록. 최신 debug plot에서 실제 추종 문제가 아니라 commanded joint trajectory 자체가 크게 흔들리는 것을 확인했다. 원인은 손교시 TCP orientation 재사용(`force_tcp_x_down: false`)과 점별 position-only MoveIt planning이 wrist/elbow IK branch를 매번 바꾸는 구조였다. `bed_config.yaml`은 `tool.force_tcp_x_down: true`로 변경했고, `planar_layer_motion.py`의 circle 실행은 점별 실행 대신 seeded IK 해를 순서대로 구해 하나의 연속 `JointTrajectory`로 보내도록 수정했다. 다음 실행 로그는 `seeded IK joint trajectory로 원 실행`이 떠야 한다.
- 2026-06-19: 이후 실행은 원 실행까지 도달하지 못하고 `시작점 상부로 Cartesian 이동`에서 fraction 95.5%가 99% 기준에 걸려 실패했다. 이는 원 궤적 품질 문제가 아니라 시작점 접근/전환 실패다. Circle start-above/start-surface 이동에 seeded IK single-pose fallback을 추가했고, 최종 안전 복귀도 Cartesian -> joint-space -> seeded IK fallback 순서로 보강했다.
- 2026-06-19: seeded IK fallback도 `error=-31`로 실패하여 no-motion `/compute_ik` 진단을 실행했다. 강제 TCP-down 자세는 `avoid_collisions`를 꺼도 circle start에서 IK 실패했고, 현재 approach TCP 자세는 start-above/첫 circle point/전체 73개 circle waypoint 모두 IK 성공했다. 따라서 출력 경로는 approach 후 실제 현재 TCP orientation을 `runtime_quat`로 읽어 고정 자세로 사용하도록 변경했다. Circle은 co-rotate TCP-down 대신 fixed `runtime_quat` + continuous seeded IK joint trajectory를 사용한다.
- 2026-06-19: 15:12 plot 확인 결과 circle까지 실행됐지만 실제 TF 원이 아직 거칠었고 command plot도 seeded IK 구간에서 흔들렸다. 현재 TCP orientation 기준 no-motion `compute_cartesian_path` 진단은 circle 전체 100% 성공(`fraction=1.0000`, 35 pts). 따라서 circle 실행은 Cartesian trajectory를 우선 사용하고, 실패할 때만 seeded IK fallback을 쓰도록 변경했다. 다음 로그는 `Cartesian trajectory로 원 실행`과 `Cartesian path planned (100.0%, ...)`가 떠야 한다.
