# 평면 적층 모션 시스템

## 뭘 만든 건지

로봇팔 TCP를 한 점에 교시하면,  
그 위치 주변에서 **지그재그 / 원형 / 나선** 패턴으로 레이어를 쌓는 모션을 자동 실행하는 시스템.

3D프린터가 G-code 없이 로봇팔로 동작하는 것과 같은 개념.

---

## 파일 3개

| 파일 | 역할 |
|------|------|
| `scripts/teach_bed.py` | 현재 TCP 위치를 bed origin으로 저장 (1회) |
| `config/bed_config.yaml` | 모든 파라미터 (크기, 레이어, 속도 등) |
| `scripts/planar_layer_motion.py` | 경로 자동 생성 + MoveIt 실행 |

---

## 워크플로

```
1. 로봇 실행 + 호밍
2. RViz에서 TCP를 원하는 위치로 이동
3. teach_bed.py 실행 → origin 저장
4. bed_config.yaml 파라미터 확인/수정
5. planar_layer_motion.py 실행
```

### 명령어

```bash
# 터미널 1
ros2 launch my_robot_ros2_control my_robot.launch.py

# 터미널 2 (호밍 끝난 후)
ros2 launch my_robot_ros2_control moveit.launch.py

# 터미널 3 (RViz에서 TCP 위치 잡은 후)
cd ~/ros2_ws/src/my_robot_ros2_control
source ~/ros2_ws/install/setup.bash

python3 scripts/teach_bed.py            # 교시 (1회)
python3 scripts/planar_layer_motion.py  # 실행
```

---

## bed_config.yaml 파라미터 설명

```yaml
bed:
  origin:
    position: [x, y, z]          # teach_bed.py가 자동으로 채워줌
    orientation: [qx, qy, qz, qw]
  width: 0.05    # 작업 영역 가로 (미터)
  height: 0.05   # 작업 영역 세로 (미터)
  stacking: world_z              # 레이어 쌓기 방향 = 월드 Z (위)

layer:
  pattern: circle                # circle / helix / raster 중 선택
  num_layers: 3                  # 총 레이어 수
  layer_height: 0.002            # 레이어 간격 (2mm)
  line_spacing: 0.01             # 래스터 라인 간격 (10mm, raster 패턴 전용)

  # circle / helix 전용
  helix_radius_scale: 0.6        # 반경 = min(width,height) × 0.6
  helix_points_per_turn: 36      # 원 1바퀴 waypoint 수 (클수록 부드러움)
  helix_start_angle_deg: 0.0     # 시작 각도
  helix_start_clearance: 0.002   # 시작점 살짝 띄우기 (mm)
  helix_enforce_world_up: true   # 기울어진 베드에서 Z 하강 방지

motion:
  z_offset: 0.02          # 교시 위치에서 추가 Z 오프셋 (위로 2cm)
  approach_height: 0.01   # 베드 위 접근 높이 (1cm)
  cartesian_max_step: 0.003  # Cartesian 보간 간격 (작을수록 정밀, 느림)
  velocity_scale: 0.27    # 속도 스케일 (0.0~1.0)
  diagonal_lift_deg: 0.0  # 레이어 간 대각선 리프트 각도 (0=수직)
```

---

## 패턴 종류

### circle (현재 설정)
```
각 레이어마다 원을 한 바퀴 그림.
Layer 1: Z=0에서 원
Layer 2: Z=layer_height에서 원
Layer 3: Z=layer_height*2에서 원
```

### helix
```
원을 그리면서 동시에 Z가 올라감.
레이어 경계 없이 연속 나선.
```

### raster (기본)
```
지그재그 (←→←→) 패턴.
폭 × 높이 직사각형 영역.
```

---

## 내부 동작 (코드가 하는 일)

```
1. bed_config.yaml 읽기
2. 교시된 origin 위치 + z_offset 적용
3. 선택된 패턴으로 waypoint 리스트 자동 생성
4. RViz에 경로 미리보기 마커 퍼블리시
5. joint-space planning으로 approach 위치 이동
6. Cartesian path로 패턴 실행
   - 실패 시 반경 자동 축소 재시도 (1.0 → 0.75 → 0.55 → 0.40)
   - 긴 경로는 구간별로 나눠서 실행
7. 완료 후 안전 높이로 복귀
```

**MoveIt 서비스 직접 사용:**
- `/compute_cartesian_path` → Cartesian 경로 계획
- `/plan_kinematic_path` → joint-space 경로 계획
- `/my_robot_arm_controller/follow_joint_trajectory` → 실행

---

## 나중에 홀센서 생기면

`teach_bed.py` 대신 홀센서가 베드를 자동 탐색해서  
`bed_config.yaml`의 `origin`을 업데이트.  
나머지 코드는 수정 없이 그대로 사용.

---

## 첫 테스트 추천 설정

```yaml
bed:
  width: 0.04
  height: 0.04
layer:
  pattern: circle
  num_layers: 2
  layer_height: 0.005
  helix_radius_scale: 0.4
motion:
  approach_height: 0.03
  velocity_scale: 0.2    # 처음엔 느리게
```
