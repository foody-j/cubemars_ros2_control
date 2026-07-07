#!/usr/bin/env python3
"""
Hall 센서 극성 검증 스크립트

목적: 6축 로봇팔의 각 Hall 센서가 트리거될 때의 극성(0 또는 1)을 확인

사용법:
    ros2 run my_robot_ros2_control verify_hall_polarity.py

작동 방식:
    1. /hall_states 토픽을 구독하여 실시간 센서 값 모니터링
    2. 사용자가 각 축의 Hall 센서를 수동으로 트리거
    3. 트리거 순간의 상태 변화(0→1 또는 1→0)를 감지
    4. 각 축의 트리거 극성을 기록 및 출력

검증 절차:
    1. 로봇 전원 켜고 Hall bridge 실행 (arduino 연결)
    2. 이 스크립트 실행
    3. 화면 지시에 따라 각 축의 Hall 센서를 손으로 트리거
       (자석을 센서에 가까이/멀리)
    4. 완료 후 결과를 URDF에 반영
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys
from collections import deque


class HallPolarityVerifier(Node):
    def __init__(self):
        super().__init__('hall_polarity_verifier')

        # 센서 상태 저장
        self.hall_states = [0, 0, 0, 0, 0, 0]
        self.prev_states = [0, 0, 0, 0, 0, 0]

        # 각 축의 트리거 극성 기록 (None: 미확인, 0/1: 확인된 극성)
        self.trigger_polarities = [None, None, None, None, None, None]

        # 상태 변화 히스토리 (노이즈 필터링용)
        self.state_history = [deque(maxlen=5) for _ in range(6)]

        # 데이터 수신 여부
        self.data_received = False

        # 토픽 구독
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/hall_states',
            self.hall_callback,
            10)

        self.get_logger().info('=' * 70)
        self.get_logger().info('🔍 Hall 센서 극성 검증 시작')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('📌 진행 방법:')
        self.get_logger().info('   1. 각 축의 Hall 센서를 수동으로 트리거하세요')
        self.get_logger().info('   2. 자석을 센서에 가까이 가져가면 상태 변화가 감지됩니다')
        self.get_logger().info('   3. 6개 축 모두 트리거되면 결과를 출력합니다')
        self.get_logger().info('')
        self.get_logger().info('⏳ /hall_states 토픽 대기 중...')

        # 주기적 상태 출력 타이머 (2초마다)
        self.timer = self.create_timer(2.0, self.print_status)

    def hall_callback(self, msg):
        """Hall 센서 데이터 수신 콜백"""
        if len(msg.data) < 6:
            self.get_logger().warn(f'⚠️  잘못된 데이터 크기: {len(msg.data)}개 (6개 필요)')
            return

        if not self.data_received:
            self.data_received = True
            self.get_logger().info('✅ Hall 센서 데이터 수신 시작!')
            self.get_logger().info('')
            self.print_initial_states(msg.data)

        # 이전 상태 저장
        self.prev_states = self.hall_states.copy()

        # 현재 상태 업데이트
        self.hall_states = list(msg.data)

        # 각 축의 상태 변화 감지
        for i in range(6):
            # 히스토리에 추가 (노이즈 필터링)
            self.state_history[i].append(self.hall_states[i])

            # 상태가 변했는지 확인
            if self.prev_states[i] != self.hall_states[i]:
                # 노이즈 필터링: 최근 3개 값이 모두 같으면 유효한 변화로 판단
                if len(self.state_history[i]) >= 3:
                    recent_values = list(self.state_history[i])[-3:]
                    if len(set(recent_values)) == 1:  # 모두 같은 값
                        self.detect_trigger(i, self.prev_states[i], self.hall_states[i])

    def detect_trigger(self, joint_idx, old_state, new_state):
        """트리거 감지 및 극성 기록"""
        # 이미 확인된 축은 스킵
        if self.trigger_polarities[joint_idx] is not None:
            return

        # 0 → 1 변화: 트리거 극성은 1
        if old_state == 0 and new_state == 1:
            self.trigger_polarities[joint_idx] = 1
            self.get_logger().info(
                f'🎯 Joint {joint_idx + 1}: 트리거 감지! (0 → 1) → 트리거 극성 = 1'
            )

        # 1 → 0 변화: 트리거 극성은 0
        elif old_state == 1 and new_state == 0:
            self.trigger_polarities[joint_idx] = 0
            self.get_logger().info(
                f'🎯 Joint {joint_idx + 1}: 트리거 감지! (1 → 0) → 트리거 극성 = 0'
            )

        # 모든 축이 확인되었는지 체크
        if all(p is not None for p in self.trigger_polarities):
            self.print_final_results()

    def print_initial_states(self, states):
        """초기 상태 출력"""
        self.get_logger().info('📊 현재 Hall 센서 초기 상태:')
        for i, state in enumerate(states):
            self.get_logger().info(f'   Joint {i + 1}: {state}')
        self.get_logger().info('')
        self.get_logger().info('👉 이제 각 축의 Hall 센서를 트리거하세요!')
        self.get_logger().info('')

    def print_status(self):
        """주기적으로 현재 상태 출력"""
        if not self.data_received:
            return

        # 아직 확인되지 않은 축 찾기
        unverified = [i + 1 for i, p in enumerate(self.trigger_polarities) if p is None]

        if unverified:
            self.get_logger().info(
                f'⏳ 대기 중... 확인 필요한 축: J{", J".join(map(str, unverified))} '
                f'(완료: {6 - len(unverified)}/6)'
            )

    def print_final_results(self):
        """최종 결과 출력"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('✅ 모든 축의 Hall 센서 극성 확인 완료!')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('📋 검증 결과:')

        for i, polarity in enumerate(self.trigger_polarities):
            self.get_logger().info(f'   Joint {i + 1}: trigger_state = {polarity}')

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('📝 URDF 설정 방법:')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')
        self.get_logger().info('파일: description/urdf/my_robot.urdf.xacro')
        self.get_logger().info('')

        # 모든 축이 같은 극성인지 확인
        if len(set(self.trigger_polarities)) == 1:
            # 모두 같은 경우
            common_polarity = self.trigger_polarities[0]
            self.get_logger().info(f'✅ 모든 축이 동일한 극성({common_polarity})을 사용합니다.')
            self.get_logger().info('')
            self.get_logger().info('URDF에 다음과 같이 설정하세요:')
            self.get_logger().info('')
            self.get_logger().info(f'  <param name="hall_trigger_state">{common_polarity}</param>')
        else:
            # 축마다 다른 경우
            self.get_logger().warn('⚠️  축마다 트리거 극성이 다릅니다!')
            self.get_logger().info('')
            self.get_logger().info('현재 코드는 모든 축에 동일한 hall_trigger_state를 사용합니다.')
            self.get_logger().info('가장 많이 사용된 극성을 권장합니다:')
            self.get_logger().info('')

            # 가장 많이 나타난 극성 찾기
            polarity_counts = {0: 0, 1: 0}
            for p in self.trigger_polarities:
                polarity_counts[p] += 1

            recommended = 0 if polarity_counts[0] > polarity_counts[1] else 1
            self.get_logger().info(f'  권장 설정: <param name="hall_trigger_state">{recommended}</param>')
            self.get_logger().info(f'  (극성 0: {polarity_counts[0]}개 축, 극성 1: {polarity_counts[1]}개 축)')
            self.get_logger().info('')
            self.get_logger().info('⚠️  다른 극성을 가진 축은 하드웨어 재배선이 필요할 수 있습니다.')

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('🏁 검증 완료! Ctrl+C로 종료하세요.')
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HallPolarityVerifier()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n\n👋 종료합니다...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
