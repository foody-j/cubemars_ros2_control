#!/usr/bin/env python3
"""Move all robot joints to logical zero with an explicit confirmation."""

import argparse
import math
import os
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "link1_1_joint",
    "link2_1_joint",
    "link3_1_joint",
    "link4_1_joint",
    "link5_1_joint",
    "link6_1_joint",
]
HOME_ZERO_LOCK = "/tmp/robot_home_zero_active"


class MoveHomeZero(Node):
    def __init__(self):
        super().__init__("move_home_zero")
        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/my_robot_arm_controller/follow_joint_trajectory",
        )

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def wait_ready(self, timeout_sec=5.0):
        deadline = time.time() + timeout_sec
        while self._joint_state is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state is None:
            self.get_logger().error("No /joint_states received.")
            return False
        return True

    def current_positions(self):
        by_name = dict(zip(self._joint_state.name, self._joint_state.position))
        missing = [name for name in JOINT_NAMES if name not in by_name]
        if missing:
            raise RuntimeError(f"/joint_states missing joints: {missing}")
        return [float(by_name[name]) for name in JOINT_NAMES]

    def move(self, duration_sec, max_speed_deg_s, segment_deg):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("JTC action server not ready.")
            return False

        current = self.current_positions()
        target = [0.0] * len(JOINT_NAMES)
        max_delta_deg = max(abs(math.degrees(value)) for value in current)
        duration_sec = max(float(duration_sec), max_delta_deg / max(float(max_speed_deg_s), 0.1))
        segment_count = max(1, int(math.ceil(max_delta_deg / max(float(segment_deg), 0.5))))

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        start = JointTrajectoryPoint()
        start.positions = current
        start.time_from_start = RosDuration(sec=0, nanosec=0)
        traj.points.append(start)

        for step in range(1, segment_count + 1):
            ratio = step / segment_count
            point = JointTrajectoryPoint()
            point.positions = [
                start + (goal - start) * ratio
                for start, goal in zip(current, target)
            ]
            t = duration_sec * ratio
            point.time_from_start = RosDuration(
                sec=int(t),
                nanosec=int((t - int(t)) * 1e9),
            )
            traj.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        goal.path_tolerance = self.loose_tolerances(position=math.radians(180.0))
        goal.goal_tolerance = self.loose_tolerances(position=math.radians(2.0))
        goal.goal_time_tolerance = RosDuration(sec=10, nanosec=0)

        self.get_logger().info(
            f"Sending action home trajectory: "
            f"{segment_count} segments, duration={duration_sec:.1f}s"
        )
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Home zero goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 20.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Home zero action timed out.")
            return False

        code = result.result.error_code
        err_deg = max(abs(math.degrees(x)) for x in self.current_positions())
        if code == 0 or err_deg <= 2.0:
            return True
        self.get_logger().error(
            f"Home zero failed. error_code={code}, max final error={err_deg:.2f}deg"
        )
        return False

    @staticmethod
    def loose_tolerances(position):
        tolerances = []
        for name in JOINT_NAMES:
            tol = JointTolerance()
            tol.name = name
            tol.position = float(position)
            tol.velocity = 0.0
            tol.acceleration = 0.0
            tolerances.append(tol)
        return tolerances


def main():
    parser = argparse.ArgumentParser(description="Move all joints to 0 deg.")
    parser.add_argument("--duration", type=float, default=60.0, help="Minimum move duration in seconds.")
    parser.add_argument(
        "--max-speed-deg-s",
        type=float,
        default=1.0,
        help="Automatically extend duration so no joint exceeds this speed.",
    )
    parser.add_argument(
        "--segment-deg",
        type=float,
        default=2.0,
        help="Maximum per-segment joint delta in degrees.",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt.")
    args = parser.parse_args()

    rclpy.init()
    node = MoveHomeZero()
    try:
        if not node.wait_ready():
            return 1
        current = node.current_positions()
        current_deg = [round(math.degrees(x), 1) for x in current]
        print("=" * 64)
        print("Move Home Zero")
        print(f"  current deg : {current_deg}")
        print("  target  deg : [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
        max_delta_deg = max(abs(math.degrees(x)) for x in current)
        effective_duration = max(args.duration, max_delta_deg / max(args.max_speed_deg_s, 0.1))
        segment_count = max(1, int(math.ceil(max_delta_deg / max(args.segment_deg, 0.5))))
        print(f"  duration    : {effective_duration:.1f}s (min={args.duration:.1f}s, max_speed={args.max_speed_deg_s:.2f}deg/s)")
        print(f"  segments    : {segment_count}  (<= {args.segment_deg:.1f}deg each)")
        print("=" * 64)
        if not args.yes:
            ans = input("Send this trajectory? (y/N): ").strip().lower()
            if ans != "y":
                print("Canceled.")
                return 0
        with open(HOME_ZERO_LOCK, "w") as f:
            f.write(f"{time.time():.3f}\n")
        try:
            ok = node.move(args.duration, args.max_speed_deg_s, args.segment_deg)
        finally:
            try:
                os.unlink(HOME_ZERO_LOCK)
            except FileNotFoundError:
                pass
        final_deg = [round(math.degrees(x), 1) for x in node.current_positions()]
        print(f"  final deg   : {final_deg}")
        print("Home zero complete." if ok else "Home zero failed.")
        return 0 if ok else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
