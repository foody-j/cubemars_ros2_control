# 로봇팔 유틸리티 스크립트

이 디렉토리는 로봇팔 운용 및 검증을 위한 유틸리티 스크립트를 포함합니다.

## 📜 스크립트 목록

### 1. verify_hall_polarity.py
**목적**: Hall 센서 트리거 극성 검증

**사용 시점**:
- 최초 로봇 설정 시
- Hall 센서 배선 변경 후
- Homing이 제대로 동작하지 않을 때

**실행 방법**:
```bash
# 1. Hall bridge 실행 (터미널 1)
ros2 launch my_robot_ros2_control my_robot.launch.py hall_port:=/dev/ttyUSB0

# 2. 검증 스크립트 실행 (터미널 2)
ros2 run my_robot_ros2_control verify_hall_polarity.py
```

**작동 원리**:
1. `/hall_states` 토픽을 구독하여 실시간 센서 값 모니터링
2. 사용자가 각 축의 Hall 센서를 수동으로 트리거
3. 트리거 순간의 상태 변화(0→1 또는 1→0)를 감지
4. 각 축의 트리거 극성(0 또는 1)을 기록
5. 최종 결과를 URDF 설정 형식으로 출력

**예상 출력**:
```
🔍 Hall 센서 극성 검증 시작
=====================================
📌 진행 방법:
   1. 각 축의 Hall 센서를 수동으로 트리거하세요
   2. 자석을 센서에 가까이 가져가면 상태 변화가 감지됩니다
   3. 6개 축 모두 트리거되면 결과를 출력합니다

⏳ /hall_states 토픽 대기 중...
✅ Hall 센서 데이터 수신 시작!

📊 현재 Hall 센서 초기 상태:
   Joint 1: 0
   Joint 2: 0
   Joint 3: 0
   Joint 4: 0
   Joint 5: 0
   Joint 6: 0

👉 이제 각 축의 Hall 센서를 트리거하세요!

🎯 Joint 1: 트리거 감지! (0 → 1) → 트리거 극성 = 1
🎯 Joint 2: 트리거 감지! (0 → 1) → 트리거 극성 = 1
🎯 Joint 3: 트리거 감지! (0 → 1) → 트리거 극성 = 1
🎯 Joint 4: 트리거 감지! (0 → 1) → 트리거 극성 = 1
🎯 Joint 5: 트리거 감지! (0 → 1) → 트리거 극성 = 1
🎯 Joint 6: 트리거 감지! (0 → 1) → 트리거 극성 = 1

✅ 모든 축의 Hall 센서 극성 확인 완료!
=====================================

📋 검증 결과:
   Joint 1: trigger_state = 1
   Joint 2: trigger_state = 1
   Joint 3: trigger_state = 1
   Joint 4: trigger_state = 1
   Joint 5: trigger_state = 1
   Joint 6: trigger_state = 1

=====================================
📝 URDF 설정 방법:
=====================================

파일: description/urdf/my_robot.urdf.xacro

✅ 모든 축이 동일한 극성(1)을 사용합니다.

URDF에 다음과 같이 설정하세요:

  <param name="hall_trigger_state">1</param>

=====================================
🏁 검증 완료! Ctrl+C로 종료하세요.
=====================================
```

**URDF 설정 반영**:
검증 완료 후 다음 파일을 수정:
```xml
<!-- 파일: description/urdf/my_robot.urdf.xacro -->
<ros2_control name="MyRobotSystem" type="system">
  <hardware>
    <plugin>my_robot_ros2_control/MyRobotSystemHardware</plugin>
    <param name="hall_trigger_state">1</param>  <!-- 검증 결과로 업데이트 -->
    ...
  </hardware>
</ros2_control>
```

수정 후 재빌드:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_ros2_control
source install/setup.bash
```

**문제 해결**:

| 문제 | 원인 | 해결 방법 |
|------|------|----------|
| 토픽 데이터 수신 안 됨 | Hall bridge 미실행 | `ros2 topic echo /hall_states`로 확인 |
| 트리거 감지 안 됨 | 센서-자석 거리 부족 | 자석을 더 가까이 가져가기 |
| 노이즈로 잘못 감지 | 전기적 노이즈 | 스크립트에 내장된 필터링 활용 |
| 축마다 극성이 다름 | 배선 불일치 | 하드웨어 재배선 또는 개별 설정 필요 |

**주의사항**:
- ⚠️ 검증 중에는 로봇이 움직이지 않습니다 (Hall 토픽만 모니터링)
- ⚠️ 각 축을 명확하게 트리거해야 정확한 결과를 얻을 수 있습니다
- ⚠️ 모든 축이 동일한 극성을 가지지 않으면 하드웨어 점검이 필요합니다

---

## 🔧 향후 추가 예정 스크립트

### verify_hall_directions.py (T1.3)
- 각 축의 홈 방향(+1 또는 -1) 검증
- 저속 homing 테스트로 안전 확인

### safe_pose_tester.py (T2.2)
- YAML 파일의 안전 포즈 테스트
- 궤적 실행 검증

### thermal_monitor.py (T5.3)
- 모터 온도 및 전류 모니터링
- 장시간 운전 시 발열 추세 기록

---

**작성일**: 2026-03-06
**버전**: 1.0
**관련 문서**: `/home/yjglaptop/ros2_ws/RUNBOOK.md`
