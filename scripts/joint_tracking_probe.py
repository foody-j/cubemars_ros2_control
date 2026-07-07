#!/usr/bin/env python3
"""
Small joint-space tracking diagnostics for the arm.

Modes:
  hold  - command one fixed joint pose and measure steady-state error
  probe - move one joint by a small +/- amplitude while other joints stay fixed

This script intentionally avoids MoveIt and Cartesian planning. It talks directly
to the FollowJointTrajectory action so command timing and /joint_states feedback
can be compared with fewer variables.
"""

import argparse
import csv
import math
import os
import time
from threading import Lock

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration as RosDuration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")

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


def load_teach_pose_rad():
    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)
    deg = cfg.get("teach", {}).get("joint_positions_deg")
    if not deg or len(deg) != 6:
        raise RuntimeError("config/bed_config.yaml teach.joint_positions_deg must contain 6 values.")
    return [math.radians(float(x)) for x in deg]


class JointTrackingProbe(Node):
    def __init__(self):
        super().__init__("joint_tracking_probe")
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            "/my_robot_arm_controller/follow_joint_trajectory",
        )
        self._lock = Lock()
        self._latest = None
        self._actual_samples = []
        self._recording = False
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 50)

    def _joint_cb(self, msg):
        pos_by_name = dict(zip(msg.name, msg.position))
        if not all(name in pos_by_name for name in JOINTS):
            return
        positions = [float(pos_by_name[name]) for name in JOINTS]
        with self._lock:
            self._latest = positions
            if self._recording:
                self._actual_samples.append([time.time()] + positions)

    def wait_ready(self):
        self.get_logger().info("Waiting for JTC action server and /joint_states...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("JTC action server not available.")
            return False
        end = time.time() + 5.0
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._lock:
                if self._latest is not None:
                    self.get_logger().info("Ready.")
                    return True
        self.get_logger().error("No /joint_states received.")
        return False

    def current_positions(self):
        with self._lock:
            return list(self._latest) if self._latest is not None else None

    def send_trajectory(self, points, timeout_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        for t, positions in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in positions]
            pt.time_from_start = ros_duration(float(t))
            goal.trajectory.points.append(pt)

        with self._lock:
            self._actual_samples = []
            self._recording = True
        command_start = time.time()

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("JTC goal rejected.")
            with self._lock:
                self._recording = False
            return False, None, command_start

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        with self._lock:
            self._recording = False

        result = result_future.result()
        if result is None:
            self.get_logger().error("JTC timed out.")
            return False, None, command_start
        self.get_logger().info(f"JTC result error_code={result.result.error_code}")
        return result.result.error_code == 0, result.result.error_code, command_start

    def run_hold(self, current, target, hold_duration, move_duration):
        self.get_logger().info(
            f"Hold test: move_duration={move_duration:.1f}s, hold_duration={hold_duration:.1f}s"
        )
        points = [
            (0.0, current),
            (move_duration, target),
            (move_duration + hold_duration, target),
        ]
        return self.send_trajectory(points, timeout_sec=move_duration + hold_duration + 5.0)

    def run_probe(self, base, joint_index, amplitude, duration):
        target_a = list(base)
        target_b = list(base)
        target_a[joint_index] += amplitude
        target_b[joint_index] -= amplitude

        self.get_logger().info(
            f"Probe test: {JOINTS[joint_index]}, amplitude={math.degrees(amplitude):.2f}deg, "
            f"duration={duration:.1f}s"
        )
        points = [
            (0.0, base),
            (duration * 0.25, target_a),
            (duration * 0.50, target_b),
            (duration * 0.75, target_a),
            (duration, base),
        ]
        return self.send_trajectory(points, timeout_sec=duration + 5.0)

    def actual_samples(self):
        with self._lock:
            return np.array(self._actual_samples, dtype=float) if self._actual_samples else np.empty((0, 7))


def interpolate_command(command_points, command_start, sample_times):
    cmd_t = np.array([command_start + p[0] for p in command_points], dtype=float)
    cmd_q = np.array([p[1] for p in command_points], dtype=float)
    out = np.empty((len(sample_times), 6), dtype=float)
    for i in range(6):
        out[:, i] = np.interp(sample_times, cmd_t, cmd_q[:, i])
    return out


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def save_outputs(prefix, actual, command_points, command_start, focus_joint):
    os.makedirs(os.path.dirname(prefix) or ".", exist_ok=True)
    actual_csv = prefix + "_actual.csv"
    command_csv = prefix + "_command.csv"
    error_csv = prefix + "_error.csv"
    plot_path = prefix + "_plot.png"

    command_rows = []
    for t, positions in command_points:
        command_rows.append([command_start + t] + list(positions))
    header = ["time"] + JOINTS
    write_csv(actual_csv, header, actual.tolist())
    write_csv(command_csv, header, command_rows)

    if len(actual) == 0:
        return actual_csv, command_csv, None, None

    cmd_interp = interpolate_command(command_points, command_start, actual[:, 0])
    err = actual[:, 1:7] - cmd_interp
    err_rows = np.column_stack((actual[:, 0], err))
    write_csv(error_csv, header, err_rows.tolist())

    t0 = min(actual[0, 0], command_start)
    t_actual = actual[:, 0] - t0
    t_cmd = np.array([command_start + p[0] - t0 for p in command_points])
    cmd_q = np.array([p[1] for p in command_points], dtype=float)

    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    j = focus_joint
    axes[0].plot(t_cmd, cmd_q[:, j], "o-", label=f"command {JOINTS[j]}")
    axes[0].plot(t_actual, actual[:, j + 1], label=f"actual {JOINTS[j]}")
    axes[0].set_ylabel("position [rad]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t_actual, np.degrees(err[:, j]), label=f"error {JOINTS[j]}")
    axes[1].axhline(2.86, color="tab:red", linestyle="--", linewidth=1.0, label="0.05rad")
    axes[1].axhline(-2.86, color="tab:red", linestyle="--", linewidth=1.0)
    axes[1].set_ylabel("error [deg]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    for i, name in enumerate(JOINTS):
        axes[2].plot(t_actual, np.degrees(err[:, i]), label=name, linewidth=1.4 if i == j else 0.8)
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("all errors [deg]")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(ncol=3, fontsize=8)

    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    return actual_csv, command_csv, error_csv, plot_path


def print_summary(actual, command_points, command_start):
    if len(actual) == 0:
        print("[WARN] No actual samples.")
        return
    cmd_interp = interpolate_command(command_points, command_start, actual[:, 0])
    err_deg = np.degrees(actual[:, 1:7] - cmd_interp)
    print("\nTracking error summary:")
    for i, name in enumerate(JOINTS):
        abs_err = np.abs(err_deg[:, i])
        print(
            f"  {name}: mean_abs={abs_err.mean():6.2f}deg  "
            f"p95={np.percentile(abs_err, 95):6.2f}deg  max={abs_err.max():6.2f}deg"
        )


def build_command_points(mode, base, joint_index, amplitude, duration):
    if mode == "hold":
        return [(0.0, base), (duration, base)]
    target_a = list(base)
    target_b = list(base)
    target_a[joint_index] += amplitude
    target_b[joint_index] -= amplitude
    return [
        (0.0, base),
        (duration * 0.25, target_a),
        (duration * 0.50, target_b),
        (duration * 0.75, target_a),
        (duration, base),
    ]


def main():
    parser = argparse.ArgumentParser(description="Small joint-space tracking probe.")
    parser.add_argument("--mode", choices=["hold", "probe"], default="hold")
    parser.add_argument("--joint", type=int, default=3, help="1-based joint index for probe mode")
    parser.add_argument("--amplitude-deg", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=12.0, help="Hold duration or full probe duration in seconds")
    parser.add_argument("--move-duration", type=float, default=20.0, help="Ramp time to target pose for hold mode")
    parser.add_argument("--teach-pose", action="store_true", help="Use bed_config teach.joint_positions_deg as base pose")
    parser.add_argument("--output-prefix", default=os.path.expanduser("~/joint_tracking_probe"))
    parser.add_argument("--yes", action="store_true", help="Skip confirmation prompt")
    args = parser.parse_args()

    rclpy.init()
    node = JointTrackingProbe()
    try:
        if not node.wait_ready():
            return
        current = node.current_positions()
        base = load_teach_pose_rad() if args.teach_pose else current
        joint_index = max(0, min(5, args.joint - 1))
        amplitude = math.radians(float(args.amplitude_deg))
        if args.mode == "hold":
            command_points = [
                (0.0, current),
                (args.move_duration, base),
                (args.move_duration + args.duration, base),
            ]
        else:
            command_points = build_command_points(args.mode, base, joint_index, amplitude, args.duration)

        print("=" * 64)
        print("Joint Tracking Probe")
        print(f"  mode       : {args.mode}")
        print(f"  base pose  : {'teach.joint_positions_deg' if args.teach_pose else 'current /joint_states'}")
        if args.mode == "hold":
            print(f"  move time  : {args.move_duration:.1f}s")
            print(f"  hold time  : {args.duration:.1f}s")
        else:
            print(f"  duration   : {args.duration:.1f}s")
        if args.mode == "probe":
            print(f"  probe joint: J{joint_index + 1} ({JOINTS[joint_index]})")
            print(f"  amplitude  : +/-{args.amplitude_deg:.2f}deg")
        print("  current deg:", [round(math.degrees(x), 2) for x in current])
        print("  target  deg:", [round(math.degrees(x), 2) for x in base])
        print("=" * 64)
        if not args.yes:
            ans = input("Send this diagnostic trajectory? (y/N): ").strip().lower()
            if ans != "y":
                print("Canceled.")
                return

        if args.mode == "hold":
            ok, code, command_start = node.run_hold(current, base, args.duration, args.move_duration)
        else:
            ok, code, command_start = node.run_probe(base, joint_index, amplitude, args.duration)

        actual = node.actual_samples()
        actual_csv, command_csv, error_csv, plot_path = save_outputs(
            args.output_prefix,
            actual,
            command_points,
            command_start,
            joint_index,
        )
        print_summary(actual, command_points, command_start)
        print(f"\nJTC ok={ok} error_code={code}")
        print(f"actual CSV : {actual_csv}")
        print(f"command CSV: {command_csv}")
        if error_csv:
            print(f"error CSV  : {error_csv}")
        if plot_path:
            print(f"plot       : {plot_path}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
