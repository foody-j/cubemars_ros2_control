#!/usr/bin/env python3
"""
Joint-position-based bed teaching script.

사용법:
  1. bed_config.yaml의 teach.joint_positions_deg에 원하는 조인트 각도 입력
  2. Robot Launch 실행 후 (MoveIt 불필요):
       python3 teach_bed_by_joints.py

로봇이 지정한 조인트 위치로 이동한 뒤 TCP 위치와 자세를 bed_config.yaml에 자동 저장.
"""

import math
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
import tf2_ros

from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")

JOINT_NAMES = [
    "link1_1_joint",
    "link2_1_joint",
    "link3_1_joint",
    "link4_1_joint",
    "link5_1_joint",
    "link6_1_joint",
]

ARRIVAL_THRESHOLD_DEG = 1.0


def main():
    rclpy.init()

    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    teach_cfg = cfg.get("teach", {})
    joint_positions_deg = teach_cfg.get("joint_positions_deg")
    duration_sec = float(teach_cfg.get("duration_sec", 3.0))

    if not joint_positions_deg or len(joint_positions_deg) != 6:
        print("[ERROR] bed_config.yaml의 teach.joint_positions_deg에 6개 각도를 설정하세요.")
        rclpy.shutdown()
        return

    joint_positions_rad = [math.radians(d) for d in joint_positions_deg]

    print("=" * 52)
    print("  Teach Bed by Joints")
    print("=" * 52)
    print("  목표 조인트 각도:")
    for i, deg in enumerate(joint_positions_deg):
        print(f"    J{i + 1}: {deg:8.2f}°")
    print(f"  이동 시간: {duration_sec:.1f}s")
    print("=" * 52)

    ans = input("\n이 위치로 이동하고 베드 원점을 저장하시겠습니까? (y/N): ").strip().lower()
    if ans != "y":
        print("취소됨.")
        rclpy.shutdown()
        return

    node = Node("teach_bed_by_joints")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    action_client = ActionClient(
        node, FollowJointTrajectory,
        "/my_robot_arm_controller/follow_joint_trajectory",
    )
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    print("\n[1/3] 액션 서버 연결 대기 중...")
    if not action_client.wait_for_server(timeout_sec=10.0):
        print("[ERROR] /my_robot_arm_controller/follow_joint_trajectory 없음. Robot Launch 확인하세요.")
        rclpy.shutdown()
        return

    # 목표 trajectory 생성
    traj = JointTrajectory()
    traj.joint_names = JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = joint_positions_rad
    pt.time_from_start = RosDuration(
        sec=int(duration_sec),
        nanosec=int((duration_sec % 1) * 1e9),
    )
    traj.points = [pt]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    print(f"[2/3] 이동 중... ({duration_sec:.1f}s)")
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
        print("[WARN] 이동 타임아웃. 현재 위치로 계속 진행합니다.")
    else:
        res = result_future.result()
        if res and res.result.error_code != 0:
            print(f"[WARN] JTC 오류 코드: {res.result.error_code}. 현재 위치로 계속 진행합니다.")
        else:
            print("  ✅ 이동 완료.")

    # 도착 확인 (joint_states 한 번 읽기)
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
        print("  도착 위치:")
        all_ok = True
        for i, (name, target_rad) in enumerate(zip(JOINT_NAMES, joint_positions_rad)):
            actual_deg = math.degrees(latest_js.get(name, target_rad))
            target_deg = math.degrees(target_rad)
            diff = abs(actual_deg - target_deg)
            mark = "✅" if diff < ARRIVAL_THRESHOLD_DEG else "⚠ "
            if diff >= ARRIVAL_THRESHOLD_DEG:
                all_ok = False
            print(f"    J{i + 1}: {actual_deg:7.2f}° (목표 {target_deg:7.2f}°, 오차 {diff:.2f}°) {mark}")
        if not all_ok:
            ans2 = input("\n오차가 큽니다. 그래도 저장하시겠습니까? (y/N): ").strip().lower()
            if ans2 != "y":
                print("취소됨.")
                rclpy.shutdown()
                return

    # TF 안정화 대기 후 TCP 위치 읽기
    print("[3/3] TCP 위치 읽는 중...")
    deadline = time.time() + 2.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.05)

    try:
        transform = tf_buffer.lookup_transform("base_link", "tcp", rclpy.time.Time())
    except Exception as e:
        print(f"[ERROR] TF 읽기 실패: {e}")
        rclpy.shutdown()
        return

    t = transform.transform.translation
    r = transform.transform.rotation
    pos = [round(t.x, 5), round(t.y, 5), round(t.z, 5)]
    quat = [round(r.x, 5), round(r.y, 5), round(r.z, 5), round(r.w, 5)]

    print(f"  TCP 위치: x={pos[0]:.5f}  y={pos[1]:.5f}  z={pos[2]:.5f}")
    print(
        "  TCP 자세: "
        f"qx={quat[0]:.5f}  qy={quat[1]:.5f}  qz={quat[2]:.5f}  qw={quat[3]:.5f}"
    )

    cfg["bed"]["origin"]["position"] = pos
    cfg["bed"]["origin"]["orientation"] = quat

    with open(CONFIG_PATH, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, allow_unicode=True)

    print(f"\n  ✅ 저장 완료 → {CONFIG_PATH}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
