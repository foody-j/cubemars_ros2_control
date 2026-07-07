#!/usr/bin/env python3
"""
Park robot at a gravity-safe pose before power-off or restart.

사용법:
  1. safe_poses.yaml의 park_pose.joint_positions를 안전한 자세로 설정 (단위: radian)
  2. Robot Launch 실행 중 상태에서:
       python3 park_robot.py

로봇이 park pose로 이동 완료된 뒤 스크립트가 종료됨.
이후 Robot Launch를 종료해도 안전함.
"""

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/safe_poses.yaml")

JOINT_NAMES = [
    "link1_1_joint",
    "link2_1_joint",
    "link3_1_joint",
    "link4_1_joint",
    "link5_1_joint",
    "link6_1_joint",
]

ARRIVAL_THRESHOLD_DEG = 2.0


def main():
    rclpy.init()

    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    park_cfg = cfg.get("safe_poses", {}).get("park_pose", {})
    joint_positions_rad = park_cfg.get("joint_positions")
    duration_sec = float(park_cfg.get("duration_sec", 4.0))

    if not joint_positions_rad or len(joint_positions_rad) != 6:
        print("[ERROR] safe_poses.yaml의 park_pose.joint_positions (6개)를 설정하세요.")
        rclpy.shutdown()
        return

    joint_positions_deg = [math.degrees(r) for r in joint_positions_rad]

    print("=" * 52)
    print("  Park Robot")
    print("=" * 52)
    print("  목표 Park Pose:")
    for i, (deg, rad) in enumerate(zip(joint_positions_deg, joint_positions_rad)):
        print(f"    J{i + 1}: {deg:8.2f}°  ({rad:.4f} rad)")
    print(f"  이동 시간: {duration_sec:.1f}s")
    print("=" * 52)
    print("\n  ⚠  이동 완료 후 Robot Launch를 종료하세요.")

    ans = input("\nPark 위치로 이동하시겠습니까? (y/N): ").strip().lower()
    if ans != "y":
        print("취소됨.")
        rclpy.shutdown()
        return

    node = Node("park_robot")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    action_client = ActionClient(
        node, FollowJointTrajectory,
        "/my_robot_arm_controller/follow_joint_trajectory",
    )

    print("\n[1/2] 액션 서버 연결 대기 중...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("[ERROR] /my_robot_arm_controller/follow_joint_trajectory 없음. Robot Launch 확인하세요.")
        rclpy.shutdown()
        return

    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = [float(r) for r in joint_positions_rad]
    pt.time_from_start = RosDuration(
        sec=int(duration_sec),
        nanosec=int((duration_sec % 1) * 1e9),
    )
    traj.points = [pt]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    print(f"[2/2] Park 위치로 이동 중... ({duration_sec:.1f}s)")
    future = action_client.send_goal_async(goal)
    while not future.done():
        executor.spin_once(timeout_sec=0.05)

    gh = future.result()
    if gh is None or not gh.accepted:
        print("[ERROR] 목표 거부됨. 로봇이 READY 상태인지 확인하세요.")
        rclpy.shutdown()
        return

    result_future = gh.get_result_async()
    timeout = duration_sec + 8.0
    t0 = time.time()
    while not result_future.done() and (time.time() - t0) < timeout:
        executor.spin_once(timeout_sec=0.05)

    if not result_future.done():
        print("[WARN] 타임아웃. 현재 위치를 확인하세요.")
    else:
        res = result_future.result()
        if res and res.result.error_code != 0:
            print(f"[WARN] JTC 오류 코드: {res.result.error_code}")
        else:
            print("  ✅ Park 완료.")

    # 도착 위치 확인
    latest_js = {}

    def js_cb(msg):
        for name, pos in zip(msg.name, msg.position):
            latest_js[name] = pos

    sub = node.create_subscription(JointState, "/joint_states", js_cb, 1)
    deadline = time.time() + 2.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if all(n in latest_js for n in JOINT_NAMES):
            break
    node.destroy_subscription(sub)

    if latest_js:
        print("\n  최종 위치:")
        for i, (name, target_rad) in enumerate(zip(JOINT_NAMES, joint_positions_rad)):
            actual_deg = math.degrees(latest_js.get(name, target_rad))
            target_deg = math.degrees(float(target_rad))
            diff = abs(actual_deg - target_deg)
            mark = "✅" if diff < ARRIVAL_THRESHOLD_DEG else "⚠ "
            print(f"    J{i + 1}: {actual_deg:7.2f}° (목표 {target_deg:7.2f}°, 오차 {diff:.2f}°) {mark}")

    print("\n  ✅ Robot Launch를 종료해도 안전합니다.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
