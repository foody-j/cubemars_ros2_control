#!/usr/bin/env python3
"""
Cartesian path test script.
Usage: python3 cartesian_test.py
"""

import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose
import math
import time


def make_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    p = Pose()
    p.position.x = x
    p.position.y = y
    p.position.z = z
    p.orientation.x = qx
    p.orientation.y = qy
    p.orientation.z = qz
    p.orientation.w = qw
    return p


def main():
    rclpy.init()
    node = rclpy.create_node("cartesian_test")

    moveit = MoveItPy(node_name="cartesian_test")
    arm = moveit.get_planning_component("arm")

    # 현재 상태 출력
    arm.set_start_state_to_current_state()
    robot_state = moveit.get_robot_state()
    print("[INFO] MoveIt connected. Robot state ready.")

    # 현재 TCP 위치 확인
    robot_model = moveit.get_robot_model()
    print(f"[INFO] Planning group: arm | Tip link: tcp")

    input("\n▶ Enter를 누르면 현재 위치에서 Z+5cm 이동합니다...")

    # 현재 포즈 가져오기
    arm.set_start_state_to_current_state()
    current_pose = arm.get_current_pose()
    print(f"[INFO] Current TCP: x={current_pose.position.x:.4f}, "
          f"y={current_pose.position.y:.4f}, z={current_pose.position.z:.4f}")

    # 목표: Z방향으로 5cm 위
    target = make_pose(
        x=current_pose.position.x,
        y=current_pose.position.y,
        z=current_pose.position.z + 0.05,
        qx=current_pose.orientation.x,
        qy=current_pose.orientation.y,
        qz=current_pose.orientation.z,
        qw=current_pose.orientation.w,
    )

    waypoints = [target]

    with arm.plan_cartesian_path(waypoints) as plan_result:
        if plan_result:
            print("[INFO] Plan succeeded. Executing...")
            arm.execute(plan_result, controllers=[])
        else:
            print("[ERROR] Cartesian path planning failed.")

    print("[INFO] Done.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
