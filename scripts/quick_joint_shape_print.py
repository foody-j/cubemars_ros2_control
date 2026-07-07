#!/usr/bin/env python3
"""
Quick local joint-space shape printer.

This is a pragmatic fallback for getting small printed shapes while Cartesian
execution is unstable. It uses MoveIt FK only to estimate the local XY Jacobian
near the taught bed pose, then sends one slow FollowJointTrajectory directly.
"""

import argparse
import csv
import math
import os
import time
from threading import Lock, Thread

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import tf2_ros
import yaml
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")
BASE_LINK = "base_link"
TCP_LINK = "tcp"
JOINTS = [
    "link1_1_joint",
    "link2_1_joint",
    "link3_1_joint",
    "link4_1_joint",
    "link5_1_joint",
    "link6_1_joint",
]


def ros_duration(seconds):
    return RosDuration(
        sec=int(seconds),
        nanosec=int((seconds - int(seconds)) * 1e9),
    )


def load_config():
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)
    teach_deg = cfg.get("teach", {}).get("joint_positions_deg")
    if not teach_deg or len(teach_deg) != 6:
        raise RuntimeError("config/bed_config.yaml teach.joint_positions_deg must contain 6 values.")
    return cfg, np.array([math.radians(float(x)) for x in teach_deg], dtype=float)


def build_xy_offsets(pattern, radius_m, length_m, points):
    if pattern == "circle":
        theta = np.linspace(0.0, 2.0 * math.pi, points, endpoint=True)
        return np.column_stack((radius_m * np.cos(theta), radius_m * np.sin(theta)))
    if pattern == "line":
        x = np.linspace(-0.5 * length_m, 0.5 * length_m, points)
        return np.column_stack((x, np.zeros_like(x)))
    if pattern == "square":
        side = 2.0 * radius_m
        corners = np.array([
            [-0.5 * side, -0.5 * side],
            [0.5 * side, -0.5 * side],
            [0.5 * side, 0.5 * side],
            [-0.5 * side, 0.5 * side],
            [-0.5 * side, -0.5 * side],
        ])
        seg_points = max(2, points // 4)
        out = []
        for a, b in zip(corners[:-1], corners[1:]):
            for s in np.linspace(0.0, 1.0, seg_points, endpoint=False):
                out.append((1.0 - s) * a + s * b)
        out.append(corners[-1])
        return np.array(out, dtype=float)
    raise ValueError(f"Unsupported pattern: {pattern}")


class QuickJointShapeNode(Node):
    def __init__(self):
        super().__init__("quick_joint_shape_print")
        self._fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self._jtc_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/my_robot_arm_controller/follow_joint_trajectory",
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._lock = Lock()
        self._latest_joints = None
        self._joint_samples = []
        self._tf_samples = []
        self._recording = False
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)
        self._record_thread = None

    def _joint_cb(self, msg):
        pos_by_name = dict(zip(msg.name, msg.position))
        if not all(name in pos_by_name for name in JOINTS):
            return
        q = [float(pos_by_name[name]) for name in JOINTS]
        with self._lock:
            self._latest_joints = q
            if self._recording:
                self._joint_samples.append([time.time()] + q)

    def wait_ready(self):
        self.get_logger().info("Waiting for /compute_fk, JTC, and /joint_states...")
        if not self._fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/compute_fk not available.")
            return False
        if not self._jtc_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("JTC action server not available.")
            return False
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._latest_joints is not None:
                    return True
        self.get_logger().error("No /joint_states received.")
        return False

    def fk_xyz(self, q):
        req = GetPositionFK.Request()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [TCP_LINK]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q]

        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        resp = future.result()
        if resp is None or resp.error_code.val != 1 or not resp.pose_stamped:
            raise RuntimeError(f"FK failed: {None if resp is None else resp.error_code.val}")
        p = resp.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z], dtype=float)

    def local_xyz_jacobian(self, q0, active_indices, eps_rad):
        base = self.fk_xyz(q0)
        j = np.zeros((3, len(active_indices)), dtype=float)
        for col, idx in enumerate(active_indices):
            qp = np.array(q0, dtype=float)
            qm = np.array(q0, dtype=float)
            qp[idx] += eps_rad
            qm[idx] -= eps_rad
            fp = self.fk_xyz(qp)
            fm = self.fk_xyz(qm)
            j[:, col] = (fp - fm) / (2.0 * eps_rad)
        return base, j

    def start_recording(self, rate_hz=20.0):
        with self._lock:
            self._joint_samples = []
            self._tf_samples = []
            self._recording = True

        def worker():
            period = 1.0 / rate_hz
            while rclpy.ok():
                with self._lock:
                    if not self._recording:
                        break
                try:
                    tf = self._tf_buffer.lookup_transform(BASE_LINK, TCP_LINK, rclpy.time.Time())
                    with self._lock:
                        if self._recording:
                            self._tf_samples.append([
                                time.time(),
                                tf.transform.translation.x,
                                tf.transform.translation.y,
                                tf.transform.translation.z,
                            ])
                except Exception:
                    pass
                time.sleep(period)

        self._record_thread = Thread(target=worker, daemon=True)
        self._record_thread.start()

    def stop_recording(self):
        with self._lock:
            self._recording = False
        if self._record_thread is not None:
            self._record_thread.join(timeout=1.0)
            self._record_thread = None

    def send_joint_trajectory(self, q_points, duration_sec):
        traj = JointTrajectory()
        traj.joint_names = JOINTS
        n = len(q_points)
        for i, q in enumerate(q_points):
            t = duration_sec * i / max(1, n - 1)
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.time_from_start = ros_duration(t)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.start_recording()
        future = self._jtc_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.stop_recording()
            self.get_logger().error("JTC goal rejected.")
            return False, None

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 8.0)
        self.stop_recording()
        result = result_future.result()
        if result is None:
            self.get_logger().error("JTC timed out.")
            return False, None
        self.get_logger().info(f"JTC result error_code={result.result.error_code}")
        return result.result.error_code == 0, result.result.error_code

    def samples(self):
        with self._lock:
            joints = np.array(self._joint_samples, dtype=float) if self._joint_samples else np.empty((0, 7))
            tf = np.array(self._tf_samples, dtype=float) if self._tf_samples else np.empty((0, 4))
        return joints, tf


def save_outputs(prefix, q_points, predicted_xyz, joints, tf):
    command_csv = prefix + "_command_joints.csv"
    predicted_csv = prefix + "_predicted_xyz.csv"
    actual_joints_csv = prefix + "_actual_joints.csv"
    actual_tf_csv = prefix + "_actual_tf.csv"
    plot_path = prefix + "_plot.png"

    with open(command_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index"] + JOINTS)
        for i, q in enumerate(q_points):
            writer.writerow([i] + list(q))
    with open(predicted_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y", "z"])
        for i, p in enumerate(predicted_xyz):
            writer.writerow([i] + list(p))
    with open(actual_joints_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time"] + JOINTS)
        writer.writerows(joints.tolist())
    with open(actual_tf_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "x", "y", "z"])
        writer.writerows(tf.tolist())

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(predicted_xyz[:, 0], predicted_xyz[:, 1], "o-", label="FK predicted", linewidth=1.2, markersize=2)
    if len(tf) > 0:
        ax.plot(tf[:, 1], tf[:, 2], label="actual TF", linewidth=2.0)
    ax.set_aspect("equal", "box")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return command_csv, predicted_csv, actual_joints_csv, actual_tf_csv, plot_path


def main():
    parser = argparse.ArgumentParser(description="Quick local joint-space shape printer.")
    parser.add_argument("--pattern", choices=["circle", "line", "square"], default="circle")
    parser.add_argument("--radius-mm", type=float, default=3.0, help="Circle/square half-size")
    parser.add_argument("--length-mm", type=float, default=6.0, help="Line length")
    parser.add_argument("--points", type=int, default=80)
    parser.add_argument("--duration", type=float, default=40.0)
    parser.add_argument("--approach-duration", type=float, default=30.0)
    parser.add_argument("--active-joints", default="1,2,3,5", help="1-based joints used for XY correction")
    parser.add_argument("--damping", type=float, default=0.02)
    parser.add_argument("--fk-eps-deg", type=float, default=0.5)
    parser.add_argument("--output-prefix", default=os.path.expanduser("~/quick_joint_shape"))
    parser.add_argument("--yes", action="store_true")
    args = parser.parse_args()

    _, q0 = load_config()
    active = [max(0, min(5, int(x.strip()) - 1)) for x in args.active_joints.split(",") if x.strip()]
    xy = build_xy_offsets(
        args.pattern,
        args.radius_mm / 1000.0,
        args.length_mm / 1000.0,
        max(8, int(args.points)),
    )

    rclpy.init()
    node = QuickJointShapeNode()
    try:
        if not node.wait_ready():
            return

        base_xyz, jxyz = node.local_xyz_jacobian(q0, active, math.radians(args.fk_eps_deg))
        damping_i = (args.damping ** 2) * np.eye(jxyz.shape[0])
        # Minimum-norm damped inverse: dq = J^T (J J^T + lambda^2 I)^-1 dx
        inv = jxyz.T @ np.linalg.inv(jxyz @ jxyz.T + damping_i)

        q_points = []
        predicted = []
        for offset in xy:
            # Keep nozzle height fixed while moving in local bed XY.
            dxyz = np.array([offset[0], offset[1], 0.0], dtype=float)
            dq_active = inv @ dxyz
            q = np.array(q0, dtype=float)
            for idx, dq in zip(active, dq_active):
                q[idx] += dq
            q_points.append(q)
            predicted.append(node.fk_xyz(q))
        q_points = np.array(q_points, dtype=float)
        predicted = np.array(predicted, dtype=float)

        # Add a slow approach from current state to the first shape point.
        current = node._latest_joints
        if current is None:
            print("[ERROR] No current joint state.")
            return
        approach_points = np.linspace(np.array(current, dtype=float), q_points[0], 20)
        all_points = np.vstack((approach_points, q_points))
        total_duration = float(args.approach_duration) + float(args.duration)

        print("=" * 64)
        print("Quick Joint Shape Print")
        print(f"  pattern       : {args.pattern}")
        print(f"  size          : radius={args.radius_mm:.1f}mm length={args.length_mm:.1f}mm")
        print(f"  points        : {len(q_points)} shape + {len(approach_points)} approach")
        print(f"  duration      : {args.approach_duration:.1f}s approach + {args.duration:.1f}s shape")
        print(f"  active joints : {[i + 1 for i in active]}")
        print(f"  base xyz      : {base_xyz.round(5).tolist()}")
        ranges = np.degrees(q_points.max(axis=0) - q_points.min(axis=0))
        print(f"  joint ranges  : {[round(x, 3) for x in ranges]} deg")
        print("=" * 64)
        if not args.yes:
            ans = input("Send this shape trajectory? (y/N): ").strip().lower()
            if ans != "y":
                print("Canceled.")
                return

        ok, code = node.send_joint_trajectory(all_points, total_duration)
        joints, tf = node.samples()
        command_csv, predicted_csv, actual_joints_csv, actual_tf_csv, plot_path = save_outputs(
            args.output_prefix,
            all_points,
            np.vstack((np.repeat(predicted[:1], len(approach_points), axis=0), predicted)),
            joints,
            tf,
        )
        print(f"JTC ok={ok} error_code={code}")
        print(f"command joints : {command_csv}")
        print(f"predicted xyz  : {predicted_csv}")
        print(f"actual joints  : {actual_joints_csv}")
        print(f"actual tf      : {actual_tf_csv}")
        print(f"plot           : {plot_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
