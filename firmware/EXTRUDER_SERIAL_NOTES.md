# 압출기 스텝모터 ROS2/시리얼 설계 정리

## 파일 구성

- 아두이노 펌웨어: `extruder_no_oled/extruder_no_oled.ino`
- 로컬 YAML 예시: `config/extruder_params.yaml`
- ROS2 통합 대상 패키지: `/home/yjglaptop/cubemars_ros2_control`
- ROS2 YAML 설정: `/home/yjglaptop/cubemars_ros2_control/config/extruder_params.yaml`
- ROS2 시리얼 노드: `/home/yjglaptop/cubemars_ros2_control/src/extruder_serial_node.cpp`
- ROS2 런치 파일: `/home/yjglaptop/cubemars_ros2_control/bringup/launch/extruder.launch.py`
- Operator 압출기 GUI: `/home/yjglaptop/ros2_ws/src/my_robot_operator_tools/my_robot_operator_tools/extruder_gui.py`
- 바탕화면 런처: `/home/yjglaptop/Desktop/Extruder_GUI.desktop`
- 바탕화면 런처 실행 스크립트: `/home/yjglaptop/start_extruder_gui.sh`

## 2026-06-18 현재 완료 상태

- Arduino 압출기 펌웨어는 `extruder_no_oled.ino` 기준으로 사용한다.
- Arduino/호환 보드는 CH340 계열로 감지되었고 포트는 `/dev/ttyUSB0` 기준이다.
- ROS2 설정 파일의 기본 포트도 `/dev/ttyUSB0`로 맞췄다.
- `extruder_serial_node`는 `/extruder/command`를 받아 USB serial로 Arduino에 명령을 전달한다.
- `extruder_gui`는 `/extruder/command` publish와 `/extruder/status` subscribe를 수행한다.
- Robot Launcher에 `Extruder GUI` 버튼이 추가되어 있다.
- Robot Launcher Live State에 압출기 bridge 상태, 마지막 status line, `Extruder STATUS`, `Extruder STOP ALL`이 추가되어 있다.
- Robot Launcher의 큰 STOP 버튼은 로봇 trajectory cancel 전에 압출기 `STOP A/B/C`도 보낸다.
- 바탕화면 `Extruder GUI` 아이콘이 추가되어 있다.
- `start_extruder_gui.sh`는 `extruder_serial_node`가 없으면 먼저 `ros2 launch my_robot_ros2_control extruder.launch.py`를 백그라운드 실행한 뒤 GUI를 연다.
- GUI를 통해 실제 모터가 회전하는 것까지 확인했다.
- 따라서 압출기 단독 제어 구현은 완료 상태다.
- `planar_layer_motion.py`에는 수동 prime 방식의 압출기 1차 연동이 추가되어 있다.

아직 완료되지 않은 것은 로봇 적층 동작과의 자동 시퀀스 통합이다.
2026-06-18 현재는 코드는 붙였지만, 실제 로봇팔+압출 동시 통합 테스트는 아직 하지 않았다.

## 기본 방향

아두이노가 YAML을 직접 읽지 않는다.

YAML은 PC/ROS2 쪽에서 읽고, 아두이노에는 USB 시리얼 명령만 보낸다.

```text
ROS2 YAML 파라미터
  -> extruder_serial_node
  -> USB 시리얼
  -> Arduino 펌웨어
  -> STEP/DIR 드라이버
  -> 스텝모터
```

이 구조로 가면 아두이노 코드는 단순해지고, 나중에 ROS2에서 속도/방향/시작 상태를 쉽게 바꿀 수 있다.

## 운전 모드 설계

스위치 기반 제어와 ROS2 제어를 둘 다 남기기 위해 펌웨어에 운전 모드를 둔다.

| 모드 | 값 | 의미 |
| --- | --- | --- |
| `MIXED` | `0` | 스위치와 ROS2 시리얼 명령을 모두 허용 |
| `MANUAL` | `1` | 스위치로만 시작/정지/방향 제어 |
| `ROS` | `2` | ROS2 시리얼 명령으로만 시작/정지/방향 제어 |

기본값은 `MIXED`다.

권장 사용 방식:

- 테스트/초기 배선 확인: `MANUAL`
- 개발 중 수동 버튼과 ROS2 둘 다 필요: `MIXED`
- 실제 ROS2 작업 실행: `ROS`

`MANUAL` 모드에서도 `INTERVAL_US` 설정은 허용한다. 즉 속도 기본값은 ROS2에서 넣고, 실제 시작/정지/방향은 버튼으로 할 수 있다.

## 아두이노 펌웨어 요약

`extruder_no_oled.ino`는 기존 `oled_rgb.ino`에서 OLED와 RGB 센서 처리를 제거한 버전이다.

제어 대상은 A/B/C 3개 스텝모터다.

| 모터 | ENA | DIR | PUL | RUN 스위치 | DIR 스위치 |
| --- | --- | --- | --- | --- | --- |
| A | 2 | 3 | 4 | A0 | A1 |
| B | 5 | 6 | 7 | A2 | A3 |
| C | 8 | 9 | 10 | 11 | 12 |

기본 속도값:

```cpp
DEFAULT_A_INTERVAL_US = 50;
DEFAULT_B_INTERVAL_US = 3000;
DEFAULT_C_INTERVAL_US = 50;
```

`interval_us`는 스텝 펄스 간격이다.

- 값이 작을수록 빠름
- 값이 클수록 느림
- B축은 현재 A/C보다 훨씬 느리게 설정되어 있음

스위치 동작:

- RUN 스위치: 시작/정지 토글
- DIR 스위치: 방향 토글
- 방향 스위치를 누르고 있어도 전체 루프가 멈추지 않도록 논블로킹 방식 사용

ENA 주의:

```cpp
digitalWrite(motor.enaPin, HIGH);
```

드라이버가 `ENA LOW = 활성` 타입이면 아래처럼 바꿔야 한다.

```cpp
digitalWrite(motor.enaPin, LOW);
```

## 시리얼 명령

baudrate:

```text
115200
```

모든 명령은 줄바꿈 `\n`으로 끝나는 ASCII 문자열이다.

운전 모드:

```text
MODE MIXED
MODE MANUAL
MODE ROS
MODE 0
MODE 1
MODE 2
```

상태 확인:

```text
STATUS
```

시작/정지:

```text
START A
STOP A
RUN A 1
RUN A 0
```

방향:

```text
DIR A 0
DIR A 1
```

속도:

```text
INTERVAL_US A 50
INTERVAL_US B 3000
INTERVAL_US C 50
```

모터 이름은 `A`, `B`, `C`를 사용한다.

아두이노 응답 예시:

```text
READY
OK
ERR MOTOR
ERR COMMAND
ERR INTERVAL
ERR MODE
ERR MANUAL_MODE
ERR BUFFER
MODE=MIXED
A RUN=0 DIR=0 INTERVAL_US=50
```

## YAML 파라미터

예시:

```yaml
extruder_serial_node:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baud: 115200
    control_mode: "mixed"
    read_status_on_start: true

    motors:
      a:
        interval_us: 50
        direction: 0
        run_on_start: false
      b:
        interval_us: 3000
        direction: 0
        run_on_start: false
      c:
        interval_us: 50
        direction: 0
        run_on_start: false
```

파라미터 의미:

- `port`: 아두이노 시리얼 포트
- `baud`: 펌웨어와 맞춰야 하는 시리얼 속도
- `control_mode`: `mixed`, `manual`, `ros` 중 하나
- `read_status_on_start`: 시작 설정 후 `STATUS` 명령 전송 여부
- `interval_us`: 각 모터의 스텝 펄스 간격
- `direction`: 초기 방향
- `run_on_start`: 노드 시작 시 해당 모터를 바로 돌릴지 여부

## ROS2 노드 역할

`extruder_serial_node`는 다음 일을 한다.

- YAML 파라미터 로드
- 시리얼 포트 열기
- 시작 시 `MODE`, `INTERVAL_US`, `DIR`, `RUN` 명령 전송
- `/extruder/command` 토픽으로 수동 시리얼 명령 수신
- 아두이노 응답을 `/extruder/status` 토픽으로 publish

수동 명령 예시:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'MODE ROS'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'INTERVAL_US A 100'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'START A'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'STOP A'}"
```

상태 확인:

```bash
ros2 topic echo /extruder/status
```

## 사용법

### 1. 아두이노 펌웨어 업로드

Arduino IDE에서 아래 파일을 연다.

```text
/home/yjglaptop/et_motor/extruder_no_oled/extruder_no_oled.ino
```

보드와 포트를 선택한 뒤 업로드한다.

업로드 후 시리얼 모니터를 열 경우 설정:

```text
baudrate: 115200
line ending: Newline
```

아두이노가 정상 부팅되면 다음 응답이 나온다.

```text
READY
```

### 2. 시리얼 모니터에서 단독 테스트

먼저 상태를 확인한다.

```text
STATUS
```

예상 응답:

```text
MODE=MIXED
A RUN=0 DIR=0 INTERVAL_US=50
B RUN=0 DIR=0 INTERVAL_US=3000
C RUN=0 DIR=0 INTERVAL_US=50
```

수동 스위치만 테스트하려면:

```text
MODE MANUAL
```

이 상태에서는 RUN/DIR 스위치로 모터를 조작한다.

ROS2 명령만 테스트하려면:

```text
MODE ROS
START A
STOP A
DIR A 1
INTERVAL_US A 100
START A
```

스위치와 ROS2 명령을 둘 다 쓰려면:

```text
MODE MIXED
```

### 3. 속도 조정

속도는 `INTERVAL_US`로 조정한다.

```text
INTERVAL_US A 50
INTERVAL_US A 100
INTERVAL_US A 500
```

값이 작을수록 빠르다.

주의:

- 너무 작은 값은 모터 탈조나 드라이버 입력 한계 문제가 생길 수 있다.
- 현재 펄스 폭은 `10us`다.
- 펌웨어는 `15us`보다 작은 `interval_us`를 거부한다.

### 4. ROS2 YAML 설정 수정

ROS2에서 사용할 설정 파일:

```text
/home/yjglaptop/cubemars_ros2_control/config/extruder_params.yaml
```

예를 들어 ROS2 제어 전용으로 쓰려면:

```yaml
control_mode: "ros"
```

스위치 기반 테스트를 기본으로 하려면:

```yaml
control_mode: "manual"
```

스위치와 ROS2를 같이 허용하려면:

```yaml
control_mode: "mixed"
```

포트가 다르면 `port`를 수정한다.

```yaml
port: "/dev/ttyUSB0"
```

아두이노가 `/dev/ttyUSB0`로 잡히면:

```yaml
port: "/dev/ttyUSB0"
```

### GUI 제어

Operator tools에 압출기 GUI가 추가되어 있다.

```bash
source /opt/ros/humble/setup.bash
source /home/yjglaptop/ros2_ws/install/setup.bash
ros2 run my_robot_operator_tools extruder_gui
```

런처 GUI에서도 `Extruder GUI` 버튼으로 열 수 있다.

GUI 기능:

- `MODE ROS`, `MODE MIXED`, `MODE MANUAL` 적용
- A/B/C 모터별 `INTERVAL_US` 설정
- A/B/C 모터별 방향 설정
- A/B/C 모터별 `START`/`STOP`
- `STOP ALL`
- `/extruder/status` 수신 상태 표시

주의: GUI 버튼은 `/extruder/command` 토픽에 실제 시작/정지 명령을 보낸다.

바탕화면에서 바로 실행하려면 `Extruder GUI` 아이콘을 사용한다.

실행 스크립트:

```bash
/home/yjglaptop/start_extruder_gui.sh
```

이 스크립트는 다음 순서로 동작한다.

```text
ROS Humble source
  -> cubemars_ros2_control install source
  -> ros2_ws install source
  -> extruder_serial_node가 없으면 extruder.launch.py 백그라운드 실행
  -> extruder_gui 실행
```

### 5. ROS2 노드 실행

패키지를 빌드한 뒤 환경을 source한다.

```bash
source /opt/ros/humble/setup.bash
source /home/yjglaptop/cubemars_ros2_control/install/setup.bash
```

단독 압출기 노드 실행:

```bash
ros2 launch my_robot_ros2_control extruder.launch.py
```

다른 YAML을 지정하려면:

```bash
ros2 launch my_robot_ros2_control extruder.launch.py \
  extruder_params:=/home/yjglaptop/cubemars_ros2_control/config/extruder_params.yaml
```

### 6. ROS2에서 명령 보내기

상태 구독:

```bash
ros2 topic echo /extruder/status
```

모드 변경:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'MODE ROS'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'MODE MANUAL'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'MODE MIXED'}"
```

A 모터 시작/정지:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'START A'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'STOP A'}"
```

A 모터 방향 변경:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'DIR A 1'}"
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'DIR A 0'}"
```

A 모터 속도 변경:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'INTERVAL_US A 100'}"
```

전체 상태 요청:

```bash
ros2 topic pub --once /extruder/command std_msgs/msg/String "{data: 'STATUS'}"
```

### 7. 추천 테스트 순서

1. 아두이노 업로드
2. 시리얼 모니터에서 `STATUS` 확인
3. `MODE MANUAL` 설정
4. 실제 스위치로 A/B/C 모터가 도는지 확인
5. `INTERVAL_US A 100`, `INTERVAL_US A 50`으로 속도 변화 확인
6. `MODE ROS` 설정
7. 시리얼 명령 `START A`, `STOP A` 테스트
8. ROS2 노드 실행
9. `/extruder/status` 확인
10. `/extruder/command`로 `START A`, `STOP A` 테스트

## 모드별 동작 상세

### MIXED

스위치와 ROS2 명령이 모두 상태를 바꿀 수 있다.

예를 들어 스위치로 A를 켠 뒤 ROS2에서 `STOP A`를 보내면 A가 멈춘다. 반대로 ROS2에서 `START A`를 보낸 뒤 스위치로 끌 수도 있다.

장점:

- 디버깅이 편함
- 현장에서 버튼으로 바로 개입 가능

주의:

- ROS2가 생각하는 상태와 사람이 스위치로 바꾼 실제 상태가 달라질 수 있다.
- 실제 작업 자동화에서는 상태 확인을 같이 써야 한다.

### MANUAL

시작/정지/방향은 스위치만 받는다.

ROS2에서는 `INTERVAL_US`로 속도 설정만 바꿀 수 있다.

장점:

- 안전하게 수동 테스트 가능
- ROS2 노드가 떠 있어도 실수로 모터가 시작되지 않음

### ROS

스위치 입력을 무시하고 ROS2 시리얼 명령만 받는다.

장점:

- 자동화 작업에서 제어 주체가 명확함
- 버튼 입력으로 인해 작업 중 상태가 바뀌는 것을 방지

주의:

- 비상정지는 별도 하드웨어 차단 스위치가 필요하다.

## 현재 설계에서 남은 개선점

- 고속 펄스가 더 필요하면 `digitalWrite()` 기반 루프 대신 타이머 인터럽트 기반 펄스 생성으로 바꾸는 것이 좋다.
- 실제 프린팅/압출량 제어가 필요하면 `interval_us`만이 아니라 `steps`, `duration_ms`, `feed_rate` 같은 명령을 추가하는 편이 좋다.
- ROS2 쪽은 지금 문자열 명령 기반이므로, 나중에는 서비스나 액션으로 감싸는 것이 좋다.

## 3D 프린팅 로봇팔 통합 계획

최종 목표는 로봇팔 적층 모션과 압출기 A 모터를 하나의 작업 흐름으로 묶는 것이다.

중요한 설계 원칙:

```text
Robot Launcher
  -> 실행 / 상태 확인 / 긴급 정지 / 운영 콘솔

planar_layer_motion.py 또는 job runner
  -> 실제 로봇팔 모션과 압출 시작/정지 타이밍

Extruder GUI
  -> 수동 테스트 / 속도 보정 / 방향 확인 / 디버깅
```

런처에 모든 로직을 넣지 않는다. 런처는 사람이 작업을 시작하고 상태를 보는 콘솔로 두고, 실제 프린팅 시퀀스의 책임은 적층 스크립트 또는 job runner가 갖는다.

현재 구상한 기본 시퀀스:

```text
Robot Launch
  -> MoveIt Launch
  -> Extruder serial node 준비
  -> 로봇팔 Approach 위치 이동
  -> A 모터 interval/direction 설정
  -> A 모터 START로 prime 시작
  -> 사용자가 노즐 끝 소재 도착을 보고 Enter
  -> 적층 경로 실행
  -> 완료/실패/중단/KeyboardInterrupt 시 A 모터 STOP
```

통합 후보 위치:

- `planar_layer_motion.py`: 당장 가장 직접적인 통합 위치. 적층 시작 전후로 `/extruder/command` publish.
- Robot Launcher: 압출기 상태 표시, Extruder GUI 열기, `STOP ALL` 같은 운영 보조 기능 제공. 2026-06-18 기준 1차 구현 완료.
- `job_runner_node.cpp`: 장기적으로 적층 작업을 job 단위로 만들 때 압출 제어 단계를 포함.

권장 통합 방식:

- 단기: `planar_layer_motion.py`에 `rclpy` publisher를 추가해 `MODE ROS`, `INTERVAL_US A <값>`, `START A`, `STOP A`를 보낸다.
- 중기: Robot Launcher에 `Extruder: READY/NO PORT/LAST STATUS` 같은 상태 패널을 추가하고, `/extruder/status`를 확인해 Arduino 응답이 없으면 적층을 시작하지 않도록 한다.
- 중기: 프린팅 파라미터를 YAML로 분리한다. 후보: `extruder_interval_us`, `prime_motor`, `prime_direction`, `prime_mode`, `stop_on_finish`.
- 장기: 문자열 토픽 대신 서비스/액션 API로 감싸고, job runner에서 로봇 모션과 압출기 명령을 하나의 상태 머신으로 관리한다.

권장 단계:

1. 현재 구조 유지: 로봇팔, 압출기, GUI를 따로 테스트 가능하게 둔다.
2. Robot Launcher에는 `Extruder GUI` 버튼과 상태/정지 기능만 붙인다. 1차 완료.
3. `planar_layer_motion.py`에 최소 자동화만 추가한다: approach 후 prime, Enter 후 적층, 종료/실패/중단 시 stop. 1차 구현 완료.
4. 파라미터를 YAML로 분리한다. 1차로 `bed_config.yaml`에 `extruder` 섹션 추가 완료.
5. 프린팅 workflow가 안정되면 job runner 상태 머신으로 승격한다.

현재 `bed_config.yaml` 예시:

```yaml
extruder:
  enabled: true
  motor: C
  mode: ROS
  interval_us: 1000
  direction: 0
  require_bridge: true
  stop_on_finish: true
  prime:
    manual_confirm: true
    fixed_seconds: 0.0
```

현재 `planar_layer_motion.py` 압출 흐름:

```text
Approach 위치 이동 완료
  -> MODE ROS
  -> INTERVAL_US C <bed_config.yaml 값>
  -> DIR C <bed_config.yaml 값>
  -> START C
  -> 사용자가 노즐 끝 토출 확인 후 Enter
  -> 적층 경로 실행
  -> STOP C
  -> 안전 높이 복귀
```

`atexit` cleanup도 들어가 있어, 프로세스 종료 시점에 압출이 시작된 상태라면 해당 모터에 `STOP`을 한 번 더 보내도록 했다. 그래도 실제 안전은 GUI `STOP ALL`과 하드웨어 전원 차단을 같이 사용해야 한다.

현재 실제 압출기 배선은 C 채널에 꽂혀 있다. 따라서 통합 테스트에서는 `START C`/`STOP C`가 사용된다.

## 시스템 대시보드 방향

새로운 대시보드 앱을 또 만들기보다는 현재 Robot Launcher를 상위 시스템 대시보드로 점진적으로 확장한다.

목표 역할:

```text
Robot System Dashboard / Robot Launcher
  -> Robot Launch / MoveIt Launch 상태
  -> Joint/Hall live state
  -> Extruder bridge 상태
  -> 마지막 Extruder status
  -> Logging 상태
  -> Run Stacking
  -> Extruder GUI 열기
  -> STOP / Extruder STOP ALL
```

세부 수동 조작은 전용 GUI로 유지한다.

```text
Extruder GUI: 압출기 수동 테스트/보정
Logging GUI: CAN logging 제어
Joint Jog: 조인트 수동 이동
```

즉 “대시보드 하나에서 상태를 보고 작업을 시작”하고, “세부 보정은 전용 패널에서 한다”는 구조로 간다.

2026-06-18 1차 UI 개선:

- 창 제목을 `Robot System Dashboard`로 변경.
- 상단에 Robot, MoveIt, Extruder, Logging 상태 카드를 배치.
- 기존 `Live State`를 `Live Telemetry` 패널로 정리.
- 큰 정지 버튼은 `STOP ALL`로 표시하고, 로봇 trajectory cancel과 압출기 `STOP A/B/C`를 함께 수행.
- 기존 workflow 버튼과 전용 GUI 버튼은 유지.

추후 UI 개선 후보:

- 상태 카드 색상 동적 변경: ON=green, OFF=red, checking=gray.
- Robot/Extruder/Logging 섹션을 탭 또는 좌측 네비게이션으로 분리.
- Run Stacking 진행 상태를 별도 progress 카드로 표시.
- `/extruder/status` 파싱 결과를 A/B/C 모터별 작은 상태 표시로 분리.
- 대시보드에서 현재 선택된 bed/print/extruder YAML 요약 표시.

## 다음 실제 통합 테스트 메모

전제:

- Arduino가 `/dev/ttyUSB0`로 잡혀 있어야 한다.
- `extruder_serial_node`가 떠 있어야 한다.
- Robot Launch가 떠 있어야 한다.
- MoveIt Launch가 떠 있어야 한다.
- `bed_config.yaml`의 `extruder.enabled`가 `true`여야 한다.

권장 시작 설정:

```yaml
extruder:
  enabled: true
  motor: C
  mode: ROS
  interval_us: 3000   # 첫 통합 테스트는 보수적으로 느리게 시작
  direction: 0
  require_bridge: true
  stop_on_finish: true
  prime:
    manual_confirm: true
    fixed_seconds: 0.0
```

현재 파일에는 `interval_us: 1000`이 들어가 있다. 실제 소재 토출 테스트가 빠르거나 불안하면 `3000`으로 올려서 시작한다. 안정 확인 후 `2000`, `1000`, 더 낮은 값 순서로 줄인다.

테스트 순서:

```text
1. Arduino 연결 확인: /dev/ttyUSB0
2. Robot Launcher 실행
3. Robot Launch 실행
4. MoveIt Launch 실행
5. Robot Launcher Live State에서 Extruder ON 확인
6. Extruder GUI 또는 Launcher의 Extruder STATUS로 응답 확인
7. Run Stacking 실행
8. Approach 위치 도착
9. A 모터 prime 시작
10. 노즐 끝 소재 도착 확인 후 Enter
11. 적층 경로 실행
12. 완료/중단 시 STOP A 확인
```

문제 발생 시 우선 확인:

- `Extruder: OFF`: `extruder_serial_node` 미실행 또는 ROS graph 문제.
- `/dev/ttyUSB0` 없음: Arduino 연결/케이블/리셋 문제.
- GUI status는 오는데 모터가 안 돈다: 드라이버 전원, GND 공유, PUL/DIR/ENA 배선, ENA 극성 확인.
- 소재가 늦게 나온다: `manual_confirm` 방식으로 충분히 기다린 뒤 Enter. 자동화는 나중에 fixed prime time 또는 purge 위치를 추가한다.
- 적층 중 이상 발생: Robot Launcher의 큰 STOP 또는 Extruder STOP ALL 사용. 하드웨어 전원 차단이 최종 안전 수단.

장기 job runner 시퀀스:

```text
Check robot ready
  -> Check MoveIt ready
  -> Check extruder ready
  -> Move approach
  -> Configure extruder A
  -> Prime extruder
  -> Wait operator confirm
  -> Execute layer path
  -> Stop extruder
  -> Park or finish pose
```

안전 요구:

- 적층 스크립트 예외, Ctrl-C, MoveIt 실패, trajectory 실패 시 항상 `STOP A`를 보낸다.
- GUI의 `STOP ALL`은 수동 비상 정지 보조 수단으로 둔다.
- 실제 전원 차단/비상정지는 별도 하드웨어 스위치가 필요하다.
