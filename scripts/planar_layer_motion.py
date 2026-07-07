#!/usr/bin/env python3
"""
Planar layered raster motion.
MoveItPy 없이 ROS2 서비스/액션 직접 사용.

Prerequisites:
  - teach_bed.py로 베드 원점 교시 완료
  - my_robot.launch.py + moveit.launch.py 실행 중

Usage:
  source ~/ros2_ws/install/setup.bash
  python3 planar_layer_motion.py
"""

import os
import sys
import time
import atexit
import csv
import yaml
from datetime import datetime
from multiprocessing import Process, Queue
from queue import Empty, Full
from threading import Lock, Thread
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    RobotState, RobotTrajectory,
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")
HOME_ZERO_LOCK = "/tmp/robot_home_zero_active"
EXTRUDER_COMMAND_TOPIC = "/extruder/command"

PLANNING_GROUP = "arm"
BASE_LINK = "base_link"
TCP_LINK = "tcp"
JOINT_NAMES = [
    "link1_1_joint",
    "link2_1_joint",
    "link3_1_joint",
    "link4_1_joint",
    "link5_1_joint",
    "link6_1_joint",
]


# ── Live visualization ────────────────────────────────────────────────────────

def live_plot_worker(sample_queue, bed_xy, planned_xyz, update_rate_hz):
    """별도 프로세스에서 실제 TCP XY 경로를 실시간 표시한다."""
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[시각화] 실시간 플롯 시작 실패: {e}")
        return

    bed_xy = np.asarray(bed_xy, dtype=float)
    planned_xyz = np.asarray(planned_xyz, dtype=float)
    actual_x = []
    actual_y = []

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(bed_xy[:, 0], bed_xy[:, 1], color="black", linewidth=1.5, label="physical bed")
    if planned_xyz.size:
        ax.plot(
            planned_xyz[:, 0], planned_xyz[:, 1],
            color="tab:green", linewidth=2.0, label="planned path",
        )
    actual_line, = ax.plot([], [], color="tab:purple", linewidth=1.5, label="actual TCP")
    current_point, = ax.plot([], [], "o", color="tab:red", markersize=7, label="current TCP")
    ax.set_aspect("equal", "box")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title("Live planar path")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    bounds = [bed_xy]
    if planned_xyz.size:
        bounds.append(planned_xyz[:, :2])
    xy = np.vstack(bounds)
    margin = 0.01
    ax.set_xlim(float(xy[:, 0].min() - margin), float(xy[:, 0].max() + margin))
    ax.set_ylim(float(xy[:, 1].min() - margin), float(xy[:, 1].max() + margin))
    fig.tight_layout()
    fig.show()

    pause_sec = 1.0 / max(float(update_rate_hz), 1.0)
    running = True
    while running and plt.fignum_exists(fig.number):
        updated = False
        while True:
            try:
                item = sample_queue.get_nowait()
            except Empty:
                break
            if item is None:
                running = False
                break
            actual_x.append(float(item[0]))
            actual_y.append(float(item[1]))
            updated = True
        if updated:
            actual_line.set_data(actual_x, actual_y)
            current_point.set_data([actual_x[-1]], [actual_y[-1]])
        plt.pause(pause_sec)

    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.close(fig)


# ── Math ─────────────────────────────────────────────────────────────────────

def quat_to_rot(q):
    """[qx, qy, qz, qw] → 3×3 rotation matrix."""
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ])


def rot_to_quat(R):
    """3×3 rotation matrix → [qx, qy, qz, qw]"""
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return [
            (R[2,1] - R[1,2]) * s,
            (R[0,2] - R[2,0]) * s,
            (R[1,0] - R[0,1]) * s,
            0.25 / s,
        ]
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return [0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s]
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return [(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return [(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s]


def make_corotating_quat(approach_axis, theta):
    """
    TCP approach 방향을 유지하면서 원의 각도에 따라 자유 회전하는 quaternion.
    approach_axis: TCP X (아래를 향하는 단위벡터)
    theta: 원 위의 각도 (rad)
    → J6이 원과 함께 자연스럽게 회전 → 리밋 회피
    """
    tcp_x = approach_axis / np.linalg.norm(approach_axis)
    # 반경 방향 벡터 (tcp_x에 수직인 성분만)
    radial = np.array([np.cos(theta), np.sin(theta), 0.0])
    radial -= np.dot(radial, tcp_x) * tcp_x
    norm = np.linalg.norm(radial)
    if norm < 1e-6:
        radial = np.array([np.cos(theta + 0.01), np.sin(theta + 0.01), 0.0])
        radial -= np.dot(radial, tcp_x) * tcp_x
        norm = np.linalg.norm(radial)
    tcp_y = radial / norm
    tcp_z = np.cross(tcp_x, tcp_y)
    R = np.column_stack([tcp_x, tcp_y, tcp_z])
    return rot_to_quat(R)


def make_tcp_x_down_quat(yaw_hint=np.array([1.0, 0.0, 0.0]), down_axis=np.array([0.0, 0.0, -1.0])):
    """TCP local X를 down_axis로 세우고, TCP local Y는 yaw_hint 쪽에 가깝게 둔다."""
    tcp_x = np.array(down_axis, dtype=float)
    tcp_x = tcp_x / np.linalg.norm(tcp_x)
    tcp_y = np.array(yaw_hint, dtype=float)
    tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    if np.linalg.norm(tcp_y) < 1e-6:
        tcp_y = np.array([1.0, 0.0, 0.0])
        tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    if np.linalg.norm(tcp_y) < 1e-6:
        tcp_y = np.array([0.0, 1.0, 0.0])
        tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    tcp_y = tcp_y / np.linalg.norm(tcp_y)
    tcp_z = np.cross(tcp_x, tcp_y)
    tcp_z = tcp_z / np.linalg.norm(tcp_z)
    R = np.column_stack([tcp_x, tcp_y, tcp_z])
    return rot_to_quat(R)


def normalize_vec(v, label):
    v = np.array(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError(f"{label} vector is too small")
    return v / n


def load_bed_frame(cfg):
    bed = cfg["bed"]
    cal = bed.get("calibration", {})
    if not bool(cal.get("enabled", False)):
        return None

    origin = np.array(cal.get("origin", bed["origin"]["position"]), dtype=float)
    axis_u = normalize_vec(cal["axis_u"], "bed.calibration.axis_u")
    axis_v = np.array(cal["axis_v"], dtype=float)
    axis_v = axis_v - np.dot(axis_v, axis_u) * axis_u
    axis_v = normalize_vec(axis_v, "bed.calibration.axis_v")

    if str(cal.get("z_mode", "")).lower() == "flat_average":
        normal = np.array([0.0, 0.0, 1.0])
    else:
        normal = normalize_vec(np.cross(axis_u, axis_v), "bed.calibration.normal")
    if "normal" in cal and str(cal.get("z_mode", "")).lower() != "flat_average":
        configured_normal = normalize_vec(cal["normal"], "bed.calibration.normal")
        if np.dot(normal, configured_normal) < 0.0:
            normal = -normal
    if normal[2] < 0.0:
        normal = -normal

    return {
        "origin": origin,
        "axis_u": axis_u,
        "axis_v": axis_v,
        "normal": normal,
        "physical_width": float(cal.get("physical_width", bed.get("physical_width", bed["width"]))),
        "physical_height": float(cal.get("physical_height", bed.get("physical_height", bed["height"]))),
        "method": str(cal.get("method", "calibrated")),
    }


def make_pose(xyz, quat):
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
    p.orientation.x = float(quat[0])
    p.orientation.y = float(quat[1])
    p.orientation.z = float(quat[2])
    p.orientation.w = float(quat[3])
    return p


def pick_normal_from_axis(R, axis_name, sign):
    axis_map = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    if axis_name not in axis_map:
        raise ValueError(f"Invalid bed.normal_axis='{axis_name}'. Use x/y/z.")
    s = -1.0 if float(sign) < 0.0 else 1.0
    v = s * (R @ axis_map[axis_name])
    return v / np.linalg.norm(v)


def raster_waypoints(origin_pos, axis_u, axis_v, quat, width, height, line_spacing, z_offset, normal_vec):
    """한 레이어의 지그재그 waypoint 리스트 생성."""
    layer_origin = origin_pos + z_offset * normal_vec

    waypoints = []
    v = 0.0
    line_idx = 0
    while v <= height + 1e-6:
        u_vals = [0.0, width] if line_idx % 2 == 0 else [width, 0.0]
        for u in u_vals:
            world_pos = layer_origin + u * axis_u + v * axis_v
            waypoints.append(make_pose(world_pos, quat))
        v = round(v + line_spacing, 6)
        line_idx += 1
    return waypoints


def build_line_waypoints(start_pos, axis_u, quat, line_len):
    return [
        make_pose(start_pos, quat),
        make_pose(start_pos + line_len * axis_u, quat),
    ]


def diagonal_lift_target(pos, normal_vec, right_vec, lift_distance, angle_deg):
    """우측(+right_vec) + 상방(+normal_vec) 대각선 리프트 목표점."""
    th = np.deg2rad(float(angle_deg))
    d = np.cos(th) * normal_vec + np.sin(th) * right_vec
    d = d / np.linalg.norm(d)
    return pos + float(lift_distance) * d


def helix_waypoints(
    center_pos,
    axis_u,
    axis_v,
    normal_vec,
    quat,
    radius,
    total_turns,
    total_height,
    points_per_turn,
    start_angle_deg=45.0,
    enforce_world_up=False,
):
    """원형 나선 경로 생성: 원을 그리며 normal 방향으로 점진 상승."""
    total_turns = max(1.0, float(total_turns))
    points_per_turn = max(16, int(points_per_turn))
    total_pts = int(points_per_turn * total_turns)
    start = np.deg2rad(float(start_angle_deg))

    wps = []
    world_z = np.array([0.0, 0.0, 1.0])
    denom = float(np.dot(normal_vec, world_z))
    prev_wz = None

    for i in range(total_pts + 1):
        t = i / float(total_pts)
        th = start + 2.0 * np.pi * total_turns * t
        p = center_pos + radius * np.cos(th) * axis_u + radius * np.sin(th) * axis_v + total_height * t * normal_vec
        if enforce_world_up and prev_wz is not None and denom > 1e-6:
            wz = float(np.dot(p, world_z))
            if wz < prev_wz:
                # If helix tilt causes local downward drift in world Z, push along bed normal.
                dz = (prev_wz - wz) + 1e-5
                p = p + (dz / denom) * normal_vec
                wz = float(np.dot(p, world_z))
            prev_wz = wz
        else:
            prev_wz = float(np.dot(p, world_z))
        wps.append(make_pose(p, quat))
    return wps


def circle_waypoints(center_pos, axis_u, axis_v, quat, radius, points_per_turn,
                     start_angle_deg=45.0, approach_axis=None):
    """고정 Z 원형 경로 생성.
    approach_axis 지정 시: TCP가 원의 반경 방향으로 co-rotate → J6 리밋 회피.
    """
    points_per_turn = max(24, int(points_per_turn))
    start = np.deg2rad(float(start_angle_deg))
    wps = []
    for i in range(points_per_turn + 1):
        th = start + 2.0 * np.pi * (i / float(points_per_turn))
        p = center_pos + radius * np.cos(th) * axis_u + radius * np.sin(th) * axis_v
        if approach_axis is not None:
            radial = np.cos(th) * axis_u + np.sin(th) * axis_v
            q = make_corotating_quat_from_radial(approach_axis, radial)
        else:
            q = quat
        wps.append(make_pose(p, q))
    return wps


def make_corotating_quat_from_radial(approach_axis, radial_hint):
    """TCP approach 방향을 유지하고, TCP Y를 원 반경 방향으로 부드럽게 둔다."""
    tcp_x = np.array(approach_axis, dtype=float)
    tcp_x = tcp_x / np.linalg.norm(tcp_x)
    tcp_y = np.array(radial_hint, dtype=float)
    tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    if np.linalg.norm(tcp_y) < 1e-6:
        tcp_y = np.array([1.0, 0.0, 0.0])
        tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    if np.linalg.norm(tcp_y) < 1e-6:
        tcp_y = np.array([0.0, 1.0, 0.0])
        tcp_y -= np.dot(tcp_y, tcp_x) * tcp_x
    tcp_y = tcp_y / np.linalg.norm(tcp_y)
    tcp_z = np.cross(tcp_x, tcp_y)
    tcp_z = tcp_z / np.linalg.norm(tcp_z)
    R = np.column_stack([tcp_x, tcp_y, tcp_z])
    return rot_to_quat(R)


# ── Motion node ───────────────────────────────────────────────────────────────

class PlanarMotionNode(Node):
    def __init__(self, cartesian_min_fraction=0.99):
        super().__init__("planar_layer_motion")

        self._cartesian_min_fraction = float(cartesian_min_fraction)
        if not 0.0 < self._cartesian_min_fraction <= 1.0:
            raise ValueError(
                "motion.cartesian_min_fraction must be in the range (0.0, 1.0]."
            )

        self._cartesian_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self._plan_client = self.create_client(
            GetMotionPlan, "/plan_kinematic_path"
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik"
        )
        self._execute_client = ActionClient(
            self, FollowJointTrajectory,
            "/my_robot_arm_controller/follow_joint_trajectory"
        )
        self._joint_traj_pub = self.create_publisher(
            JointTrajectory,
            "/my_robot_arm_controller/joint_trajectory",
            10,
        )
        self._marker_pub = self.create_publisher(Marker, "/planar_layer_motion/markers", 10)
        self._extruder_pub = self.create_publisher(String, EXTRUDER_COMMAND_TOPIC, 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._executed_paths = []
        self._failed_paths = []
        self._actual_samples = []
        self._actual_segments = []
        self._actual_lock = Lock()
        self._record_actual = False
        self._actual_thread = None
        self._live_plot_queue = None
        self._live_plot_process = None
        self._joint_lock = Lock()
        self._latest_joint_positions = None
        self._joint_samples = []
        self._commanded_joint_samples = []
        self._record_joints = False
        self._bed_floor_origin = None
        self._bed_floor_normal = None
        self._min_bed_clearance = 0.0
        self._bed_floor_tolerance = 0.0005
        self.create_subscription(JointState, "/joint_states", self._record_joint_state, 50)

        self.get_logger().info("Waiting for MoveIt services...")
        self._cartesian_client.wait_for_service(timeout_sec=10.0)
        self._plan_client.wait_for_service(timeout_sec=10.0)
        self._ik_client.wait_for_service(timeout_sec=10.0)
        self._execute_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("MoveIt ready.")

    def configure_bed_floor(self, origin_pos, normal_vec, min_clearance=0.0, tolerance=0.0005):
        self._bed_floor_origin = np.array(origin_pos, dtype=float)
        normal = np.array(normal_vec, dtype=float)
        self._bed_floor_normal = normal / np.linalg.norm(normal)
        self._min_bed_clearance = float(min_clearance)
        self._bed_floor_tolerance = float(tolerance)
        self.get_logger().info(
            "Bed floor guard enabled: "
            f"min_clearance={self._min_bed_clearance*1000:.1f}mm, "
            f"tolerance={self._bed_floor_tolerance*1000:.1f}mm"
        )

    def _pose_clearance(self, pose):
        if self._bed_floor_origin is None or self._bed_floor_normal is None:
            return None
        xyz = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
        return float(np.dot(xyz - self._bed_floor_origin, self._bed_floor_normal))

    def _check_above_bed_floor(self, label, poses):
        if self._bed_floor_origin is None or self._bed_floor_normal is None:
            return True
        min_allowed = self._min_bed_clearance - self._bed_floor_tolerance
        min_seen = None
        bad_index = None
        for i, pose in enumerate(poses):
            clearance = self._pose_clearance(pose)
            if clearance is None:
                continue
            if min_seen is None or clearance < min_seen:
                min_seen = clearance
                bad_index = i
        if min_seen is not None and min_seen < min_allowed:
            self.get_logger().error(
                f"{label}: target goes below bed floor. "
                f"min_clearance={min_seen*1000:.2f}mm, "
                f"allowed={self._min_bed_clearance*1000:.2f}mm "
                f"(pose index {bad_index}). Abort."
            )
            return False
        return True

    def wait_for_extruder_bridge(self, timeout_sec=2.0):
        deadline = time.time() + float(timeout_sec)
        while time.time() < deadline and rclpy.ok():
            if self.count_subscribers(EXTRUDER_COMMAND_TOPIC) > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.count_subscribers(EXTRUDER_COMMAND_TOPIC) > 0

    def send_extruder_command(self, line, settle_sec=0.05):
        msg = String()
        msg.data = str(line)
        self._extruder_pub.publish(msg)
        self.get_logger().info(f"extruder << {line}")
        rclpy.spin_once(self, timeout_sec=0.0)
        if settle_sec > 0.0:
            time.sleep(float(settle_sec))

    def start_extruder_prime(self, extruder_cfg):
        motor = str(extruder_cfg.get("motor", "A")).upper()
        mode = str(extruder_cfg.get("mode", "ROS")).upper()
        interval_us = int(extruder_cfg.get("interval_us", 1000))
        direction = int(extruder_cfg.get("direction", 0))
        require_bridge = bool(extruder_cfg.get("require_bridge", True))

        if require_bridge and not self.wait_for_extruder_bridge(timeout_sec=2.0):
            raise RuntimeError(
                "extruder_serial_node가 /extruder/command를 구독하지 않습니다. "
                "Robot Launch 또는 Extruder GUI 런처를 확인하세요."
            )

        self.send_extruder_command(f"MODE {mode}")
        self.send_extruder_command(f"INTERVAL_US {motor} {interval_us}")
        self.send_extruder_command(f"DIR {motor} {direction}")
        self.send_extruder_command(f"START {motor}")
        return motor

    def stop_extruder_motor(self, motor):
        self.send_extruder_command(f"STOP {str(motor).upper()}")

    def _make_marker(self, marker_id, marker_type, color_rgba, scale_xyz):
        m = Marker()
        m.header.frame_id = BASE_LINK
        m.ns = "planar_layer_motion"
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = scale_xyz
        m.color.r, m.color.g, m.color.b, m.color.a = color_rgba
        return m

    def publish_path_preview(self, waypoints):
        if not waypoints:
            return
        m = self._make_marker(
            marker_id=1,
            marker_type=Marker.LINE_STRIP,
            color_rgba=(0.1, 0.8, 1.0, 0.9),
            scale_xyz=(0.002, 0.0, 0.0),
        )
        for wp in waypoints:
            p = Point()
            p.x, p.y, p.z = wp.position.x, wp.position.y, wp.position.z
            m.points.append(p)
        self._marker_pub.publish(m)

    def publish_progress_point(self, pose):
        m = self._make_marker(
            marker_id=2,
            marker_type=Marker.SPHERE,
            color_rgba=(1.0, 0.2, 0.1, 0.95),
            scale_xyz=(0.01, 0.01, 0.01),
        )
        m.pose = pose
        self._marker_pub.publish(m)

    def _waypoints_to_xyz(self, waypoints):
        return np.array(
            [[wp.position.x, wp.position.y, wp.position.z] for wp in waypoints],
            dtype=float,
        )

    def record_executed_path(self, label, waypoints):
        if waypoints:
            self._executed_paths.append((label, self._waypoints_to_xyz(waypoints)))

    def record_failed_path(self, label, waypoints):
        if waypoints:
            self._failed_paths.append((label, self._waypoints_to_xyz(waypoints)))

    def _record_joint_state(self, msg):
        pos_by_name = dict(zip(msg.name, msg.position))
        if not all(name in pos_by_name for name in JOINT_NAMES):
            return
        positions = [float(pos_by_name[name]) for name in JOINT_NAMES]
        with self._joint_lock:
            self._latest_joint_positions = positions
            if self._record_joints:
                self._joint_samples.append([time.time()] + positions)

    def start_actual_recording(self, rate_hz=20.0):
        if self._actual_thread is not None:
            return
        self._record_actual = True
        self._record_joints = True

        def worker():
            period = 1.0 / float(rate_hz)
            while self._record_actual and rclpy.ok():
                try:
                    tf = self._tf_buffer.lookup_transform(BASE_LINK, TCP_LINK, rclpy.time.Time())
                    sample = [
                        time.time(),
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z,
                    ]
                    with self._actual_lock:
                        self._actual_samples.append(sample)
                    if self._live_plot_queue is not None:
                        try:
                            self._live_plot_queue.put_nowait(sample[1:4])
                        except Full:
                            pass
                except Exception:
                    pass
                time.sleep(period)

        self._actual_thread = Thread(target=worker, daemon=True)
        self._actual_thread.start()

    def start_live_plot(self, bed_xy, planned_waypoints, update_rate_hz=10.0):
        if self._live_plot_process is not None:
            return
        planned_xyz = (
            self._waypoints_to_xyz(planned_waypoints)
            if planned_waypoints else np.empty((0, 3))
        )
        self._live_plot_queue = Queue(maxsize=1000)
        self._live_plot_process = Process(
            target=live_plot_worker,
            args=(
                self._live_plot_queue,
                np.asarray(bed_xy),
                planned_xyz,
                update_rate_hz,
            ),
            daemon=True,
        )
        self._live_plot_process.start()
        self.get_logger().info("Live TCP path window started.")

    def stop_live_plot(self):
        if self._live_plot_process is None:
            return
        try:
            self._live_plot_queue.put_nowait(None)
        except Full:
            pass
        self._live_plot_process.join(timeout=1.0)
        if self._live_plot_process.is_alive():
            self._live_plot_process.terminate()
            self._live_plot_process.join(timeout=0.5)
        self._live_plot_process = None
        self._live_plot_queue = None

    def stop_actual_recording(self):
        self._record_actual = False
        self._record_joints = False
        if self._actual_thread is not None:
            self._actual_thread.join(timeout=1.0)
            self._actual_thread = None

    def get_actual_path(self):
        with self._actual_lock:
            if not self._actual_samples:
                return np.empty((0, 4))
            return np.array(self._actual_samples, dtype=float)

    def clear_path_records(self):
        self._executed_paths.clear()
        self._failed_paths.clear()
        self._actual_segments.clear()
        self._commanded_joint_samples.clear()
        with self._actual_lock:
            self._actual_samples.clear()
        with self._joint_lock:
            self._joint_samples.clear()

    def _actual_sample_count(self):
        with self._actual_lock:
            return len(self._actual_samples)

    def _record_actual_segment(self, label, start_index):
        with self._actual_lock:
            segment = self._actual_samples[start_index:]
        if len(segment) >= 2:
            self._actual_segments.append((label, np.array(segment, dtype=float)))

    def _record_commanded_joint_trajectory(self, trajectory, start_time):
        names = list(trajectory.joint_names)
        name_to_index = {name: i for i, name in enumerate(names)}
        for pt in trajectory.points:
            if len(pt.positions) < len(names):
                continue
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            row = [start_time + t]
            valid = True
            for joint_name in JOINT_NAMES:
                idx = name_to_index.get(joint_name)
                if idx is None or idx >= len(pt.positions):
                    valid = False
                    break
                row.append(float(pt.positions[idx]))
            if valid:
                self._commanded_joint_samples.append(row)

    def save_joint_tracking_debug(self, output_prefix):
        actual_csv = output_prefix + "_actual_joints.csv"
        command_csv = output_prefix + "_commanded_joints.csv"
        plot_path = output_prefix + "_joint_tracking.png"

        with self._joint_lock:
            actual = np.array(self._joint_samples, dtype=float) if self._joint_samples else np.empty((0, 7))
        commanded = (
            np.array(self._commanded_joint_samples, dtype=float)
            if self._commanded_joint_samples else np.empty((0, 7))
        )
        if len(actual) > 0:
            actual = actual[np.argsort(actual[:, 0], kind="stable")]
        if len(commanded) > 0:
            commanded = commanded[np.argsort(commanded[:, 0], kind="stable")]

        header = ["time"] + JOINT_NAMES
        for path, data in ((actual_csv, actual), (command_csv, commanded)):
            with open(path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(header)
                writer.writerows(data.tolist())

        if len(actual) == 0 and len(commanded) == 0:
            self.get_logger().warn("No joint tracking samples to plot.")
            return actual_csv, command_csv, None

        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(f"Could not import matplotlib for joint tracking plot: {e}")
            return actual_csv, command_csv, None

        t0_candidates = []
        if len(actual) > 0:
            t0_candidates.append(actual[0, 0])
        if len(commanded) > 0:
            t0_candidates.append(commanded[0, 0])
        t0 = min(t0_candidates) if t0_candidates else 0.0

        fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex=True)
        axes = axes.flatten()
        for i, joint_name in enumerate(JOINT_NAMES):
            ax = axes[i]
            if len(commanded) > 0:
                ax.plot(
                    commanded[:, 0] - t0,
                    commanded[:, i + 1],
                    color="tab:blue",
                    linewidth=1.6,
                    label="command",
                )
            if len(actual) > 0:
                ax.plot(
                    actual[:, 0] - t0,
                    actual[:, i + 1],
                    color="tab:orange",
                    linewidth=1.0,
                    label="actual",
                )
            ax.set_title(joint_name)
            ax.set_ylabel("rad")
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
        axes[-1].set_xlabel("time [s]")
        axes[-2].set_xlabel("time [s]")
        fig.suptitle("Commanded joint trajectory vs actual /joint_states")
        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        return actual_csv, command_csv, plot_path

    def save_execution_plot(self, planned_waypoints, output_path):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            self.get_logger().warn(f"Could not import matplotlib for execution plot: {e}")
            return None

        planned = self._waypoints_to_xyz(planned_waypoints) if planned_waypoints else np.empty((0, 3))
        actual = self.get_actual_path()
        all_arrays = [arr for _, arr in self._executed_paths + self._failed_paths]
        if len(planned) > 0:
            all_arrays.append(planned)
        if len(actual) > 0:
            all_arrays.append(actual[:, 1:4])
        if not all_arrays:
            self.get_logger().warn("No path data to plot.")
            return None

        xyz = np.vstack(all_arrays)
        fig = plt.figure(figsize=(13, 6))
        ax3 = fig.add_subplot(1, 2, 1, projection="3d")
        ax2 = fig.add_subplot(1, 2, 2)

        if len(planned) > 0:
            ax3.plot(planned[:, 0], planned[:, 1], planned[:, 2], color="0.75", linewidth=1.0, label="planned")
            ax2.plot(planned[:, 0], planned[:, 1], color="0.75", linewidth=1.0, label="planned")

        for label, arr in self._executed_paths:
            ax3.plot(arr[:, 0], arr[:, 1], arr[:, 2], color="tab:green", linewidth=2.0)
            ax2.plot(arr[:, 0], arr[:, 1], color="tab:green", linewidth=2.0)
            ax3.scatter(arr[0, 0], arr[0, 1], arr[0, 2], color="tab:green", s=18)

        for label, arr in self._failed_paths:
            ax3.plot(arr[:, 0], arr[:, 1], arr[:, 2], color="tab:red", linestyle="--", linewidth=1.4)
            ax2.plot(arr[:, 0], arr[:, 1], color="tab:red", linestyle="--", linewidth=1.4)

        for label, segment in self._actual_segments:
            xyz_segment = segment[:, 1:4]
            ax3.plot(
                xyz_segment[:, 0], xyz_segment[:, 1], xyz_segment[:, 2],
                color="tab:purple", linewidth=2.2,
            )
            ax2.plot(xyz_segment[:, 0], xyz_segment[:, 1], color="tab:purple", linewidth=2.2)
            ax3.scatter(xyz_segment[0, 0], xyz_segment[0, 1], xyz_segment[0, 2], color="tab:purple", s=18)

        mins = xyz.min(axis=0)
        maxs = xyz.max(axis=0)
        center = (mins + maxs) / 2.0
        radius = max(float(np.max(maxs - mins)) * 0.6, 0.02)
        ax3.set_xlim(center[0] - radius, center[0] + radius)
        ax3.set_ylim(center[1] - radius, center[1] + radius)
        ax3.set_zlim(center[2] - radius, center[2] + radius)
        ax3.set_xlabel("X [m]")
        ax3.set_ylabel("Y [m]")
        ax3.set_zlabel("Z [m]")
        ax3.set_title("Executed Path 3D")

        ax2.set_aspect("equal", "box")
        ax2.grid(True, alpha=0.3)
        ax2.set_xlabel("X [m]")
        ax2.set_ylabel("Y [m]")
        ax2.set_title("Top View")

        from matplotlib.lines import Line2D
        legend = [
            Line2D([0], [0], color="0.75", lw=1.0, label="planned"),
            Line2D([0], [0], color="tab:green", lw=2.0, label="executed"),
            Line2D([0], [0], color="tab:red", lw=1.4, linestyle="--", label="failed/rejected"),
            Line2D([0], [0], color="tab:purple", lw=2.2, label="actual TF"),
        ]
        ax3.legend(handles=legend, loc="upper left")
        fig.suptitle(f"Planar layer execution - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        fig.tight_layout()
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        return output_path

    def get_current_joint_state(self):
        """최신 joint_states 한 번 수신."""
        msg = None
        def cb(m):
            nonlocal msg
            msg = m
        sub = self.create_subscription(JointState, "/joint_states", cb, 1)
        deadline = time.time() + 3.0
        while msg is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)
        return msg

    @staticmethod
    def scale_trajectory_speed(joint_trajectory, scale):
        """Stretch timing while keeping velocity/acceleration fields consistent."""
        if scale >= 1.0:
            return
        if scale <= 0.0:
            raise ValueError("velocity scale must be greater than zero")

        for pt in joint_trajectory.points:
            t = pt.time_from_start
            scaled_time = (t.sec + t.nanosec * 1e-9) / scale
            pt.time_from_start.sec = int(scaled_time)
            pt.time_from_start.nanosec = int(
                round((scaled_time - int(scaled_time)) * 1e9)
            )
            if pt.time_from_start.nanosec >= 1_000_000_000:
                pt.time_from_start.sec += 1
                pt.time_from_start.nanosec -= 1_000_000_000
            if pt.velocities:
                pt.velocities = [value * scale for value in pt.velocities]
            if pt.accelerations:
                pt.accelerations = [
                    value * scale * scale for value in pt.accelerations
                ]

    @staticmethod
    def _joint_positions_by_name(js):
        if js is None:
            return None
        pos_by_name = dict(zip(js.name, js.position))
        if not all(name in pos_by_name for name in JOINT_NAMES):
            return None
        return [float(pos_by_name[name]) for name in JOINT_NAMES]

    @staticmethod
    def _unwrap_near_previous(solution, previous):
        """Keep equivalent revolute angles close to the previous command."""
        unwrapped = []
        for value, prev in zip(solution, previous):
            delta = (float(value) - float(prev) + np.pi) % (2.0 * np.pi) - np.pi
            unwrapped.append(float(prev) + delta)
        return unwrapped

    def compute_seeded_ik_joint_path(self, waypoints):
        """Compute ordered IK solutions using the previous waypoint as seed."""
        if not self._check_above_bed_floor("compute_seeded_ik_joint_path", waypoints):
            return None

        seed_js = self.get_current_joint_state()
        prev_positions = self._joint_positions_by_name(seed_js)
        if seed_js is None or prev_positions is None:
            self.get_logger().error("Seeded IK: /joint_states missing expected joints.")
            return None

        joint_path = []
        for i, wp in enumerate(waypoints):
            req = GetPositionIK.Request()
            req.ik_request.group_name = PLANNING_GROUP
            req.ik_request.robot_state = RobotState()
            req.ik_request.robot_state.joint_state = seed_js
            req.ik_request.avoid_collisions = True
            req.ik_request.ik_link_name = TCP_LINK
            req.ik_request.pose_stamped = PoseStamped()
            req.ik_request.pose_stamped.header.frame_id = BASE_LINK
            req.ik_request.pose_stamped.pose = wp
            req.ik_request.timeout = RosDuration(sec=0, nanosec=250_000_000)

            future = self._ik_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() is None:
                self.get_logger().error(f"Seeded IK point {i}: service failed.")
                return None

            resp = future.result()
            if resp.error_code.val != 1:
                self.get_logger().error(
                    f"Seeded IK point {i}: no solution, error={resp.error_code.val}"
                )
                return None

            raw_positions = self._joint_positions_by_name(resp.solution.joint_state)
            if raw_positions is None:
                self.get_logger().error(f"Seeded IK point {i}: solution missing joints.")
                return None

            positions = self._unwrap_near_previous(raw_positions, prev_positions)
            joint_path.append(positions)

            seed_js = JointState()
            seed_js.name = list(JOINT_NAMES)
            seed_js.position = list(positions)
            prev_positions = positions

        return joint_path

    def build_joint_trajectory_from_path(self, joint_path, velocity_scale=0.5):
        js = self.get_current_joint_state()
        current = self._joint_positions_by_name(js)
        if current is None:
            self.get_logger().error("Joint trajectory: /joint_states missing expected joints.")
            return None

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        start_pt = JointTrajectoryPoint()
        start_pt.positions = list(current)
        start_pt.time_from_start = RosDuration(sec=0, nanosec=0)
        traj.points.append(start_pt)

        prev = current
        elapsed = 0.0
        speed = np.deg2rad(18.0) * max(float(velocity_scale), 0.05) / 0.12
        for positions in joint_path:
            max_delta = max(abs(p - q) for p, q in zip(positions, prev))
            dt = max(0.12, min(1.5, max_delta / max(speed, 1e-6)))
            elapsed += dt

            pt = JointTrajectoryPoint()
            pt.positions = list(positions)
            pt.time_from_start = RosDuration(
                sec=int(elapsed),
                nanosec=int((elapsed - int(elapsed)) * 1e9),
            )
            traj.points.append(pt)
            prev = positions

        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = traj
        return robot_traj

    def seeded_ik_move(self, target_pose, velocity_scale=0.5, label="seeded_ik_move"):
        """Move to one pose by IK from the current joint state, without full path planning."""
        joint_path = self.compute_seeded_ik_joint_path([target_pose])
        if joint_path is None:
            self.record_failed_path(f"{label}_ik_failed", [target_pose])
            return False

        robot_traj = self.build_joint_trajectory_from_path(joint_path, velocity_scale)
        if robot_traj is None:
            self.record_failed_path(f"{label}_trajectory_failed", [target_pose])
            return False

        self.get_logger().info(f"{label}: seeded IK move planned.")
        ok = self.execute_trajectory(robot_traj)
        if ok:
            self.record_executed_path(label, [target_pose])
        else:
            self.record_failed_path(f"{label}_execute_failed", [target_pose])
        return ok

    def compute_cartesian(self, waypoints, max_step=0.005, velocity_scale=0.5):
        """GetCartesianPath 서비스 호출."""
        if not self._check_above_bed_floor("compute_cartesian", waypoints):
            return None
        js = self.get_current_joint_state()
        if js is None:
            self.get_logger().error("No joint states received.")
            return None

        req = GetCartesianPath.Request()
        req.header.frame_id = BASE_LINK
        req.group_name = PLANNING_GROUP
        req.link_name = TCP_LINK
        req.start_state = RobotState()
        req.start_state.joint_state = js
        req.waypoints = waypoints
        req.max_step = max_step
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().error("Cartesian plan service failed.")
            return None

        result = future.result()
        fraction = float(result.fraction)
        trajectory = result.solution.joint_trajectory
        if fraction < self._cartesian_min_fraction:
            self.get_logger().error(
                f"Cartesian path incomplete ({fraction*100:.1f}% < "
                f"{self._cartesian_min_fraction*100:.1f}%). Rejecting partial trajectory."
            )
            return None
        if not trajectory.points:
            self.get_logger().error("Cartesian planner returned an empty trajectory. Abort.")
            return None

        self.get_logger().info(
            f"Cartesian path planned ({fraction*100:.1f}%, "
            f"{len(trajectory.points)} pts)."
        )

        self.scale_trajectory_speed(trajectory, velocity_scale)

        return result.solution, fraction

    def compute_cartesian_full(self, waypoints, max_step=0.005, velocity_scale=0.5):
        """허용 fraction 이상인 Cartesian path만 반환. 실패 시 None."""
        result = self.compute_cartesian(waypoints, max_step, velocity_scale)
        if result is None:
            return None
        traj, _ = result
        return traj

    def execute_trajectory(self, trajectory: RobotTrajectory):
        """JTC FollowJointTrajectory action으로 프린팅/Cartesian 경로를 실행."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory.joint_trajectory

        if not goal.trajectory.points:
            self.get_logger().error("execute_trajectory: empty trajectory.")
            return False

        future = self._execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("JTC goal rejected.")
            return False

        pts = goal.trajectory.points
        last_t = pts[-1].time_from_start
        duration = last_t.sec + last_t.nanosec * 1e-9
        timeout = duration + 5.0

        actual_start_index = self._actual_sample_count()
        command_start_time = time.time()
        self._record_commanded_joint_trajectory(goal.trajectory, command_start_time)

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        result = result_future.result()
        if result is None:
            self.get_logger().error("JTC timed out.")
            return False

        error_code = result.result.error_code
        if error_code != 0:
            self.get_logger().error(f"JTC failed. error_code={error_code}")
            self._record_actual_segment("actual_jtc_failed", actual_start_index)
            return False

        self._record_actual_segment("actual_jtc_ok", actual_start_index)
        return True

    def direct_joint_move(self, target_positions, duration_sec=30.0):
        """MoveIt을 거치지 않고 현재 조인트에서 목표 조인트로 직접 천천히 이동."""
        js = self.get_current_joint_state()
        if js is None:
            self.get_logger().error("direct_joint_move: No joint states received.")
            return False

        pos_by_name = dict(zip(js.name, js.position))
        if not all(name in pos_by_name for name in JOINT_NAMES):
            self.get_logger().error("direct_joint_move: /joint_states missing expected joints.")
            return False

        current = [float(pos_by_name[name]) for name in JOINT_NAMES]
        target = [float(x) for x in target_positions]
        if len(target) != len(JOINT_NAMES):
            self.get_logger().error("direct_joint_move: target must contain 6 joint positions.")
            return False

        max_delta_deg = max(
            abs(np.rad2deg(t - c)) for c, t in zip(current, target)
        )
        effective_duration = max(float(duration_sec), max_delta_deg / 5.0)
        segment_count = max(1, int(np.ceil(max_delta_deg / 2.0)))

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        start_pt = JointTrajectoryPoint()
        start_pt.positions = current
        start_pt.time_from_start = RosDuration(sec=0, nanosec=0)
        traj.points.append(start_pt)

        for step in range(1, segment_count + 1):
            ratio = step / segment_count
            pt = JointTrajectoryPoint()
            pt.positions = [
                c + (t - c) * ratio
                for c, t in zip(current, target)
            ]
            t_sec = effective_duration * ratio
            pt.time_from_start = RosDuration(
                sec=int(t_sec),
                nanosec=int((t_sec - int(t_sec)) * 1e9),
            )
            traj.points.append(pt)

        actual_start_index = self._actual_sample_count()
        command_start_time = time.time()
        self._record_commanded_joint_trajectory(traj, command_start_time)

        self.get_logger().info(
            "direct_joint_move: publishing joint trajectory to teach joints "
            f"over {effective_duration:.1f}s ({segment_count} segments)."
        )
        self._joint_traj_pub.publish(traj)

        deadline = time.time() + effective_duration + 15.0
        while time.time() < deadline:
            js = self.get_current_joint_state()
            if js is not None:
                pos_by_name = dict(zip(js.name, js.position))
                if all(name in pos_by_name for name in JOINT_NAMES):
                    actual = [float(pos_by_name[name]) for name in JOINT_NAMES]
                    max_err_deg = max(
                        abs(np.rad2deg(t - a))
                        for a, t in zip(actual, target)
                    )
                    if max_err_deg <= 2.0:
                        self._record_actual_segment("actual_direct_joint_ok", actual_start_index)
                        self._publish_hold_current(actual)
                        return True
            rclpy.spin_once(self, timeout_sec=0.1)

        js = self.get_current_joint_state()
        if js is not None:
            pos_by_name = dict(zip(js.name, js.position))
            if all(name in pos_by_name for name in JOINT_NAMES):
                actual = [float(pos_by_name[name]) for name in JOINT_NAMES]
                max_err_deg = max(
                    abs(np.rad2deg(t - a))
                    for a, t in zip(actual, target)
                )
                self.get_logger().error(
                    f"direct_joint_move: target not reached. max_err={max_err_deg:.2f}deg"
                )
                self._publish_hold_current(actual)
        self._record_actual_segment("actual_direct_joint_failed", actual_start_index)
        return False

    def _publish_hold_current(self, positions):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        p0 = JointTrajectoryPoint()
        p0.positions = list(positions)
        p0.time_from_start = RosDuration(sec=0, nanosec=0)
        traj.points.append(p0)

        p1 = JointTrajectoryPoint()
        p1.positions = list(positions)
        p1.time_from_start = RosDuration(sec=1, nanosec=0)
        traj.points.append(p1)

        self._joint_traj_pub.publish(traj)

    def joint_space_move(self, target_pose, velocity_scale=0.5):
        """Joint-space planning으로 Pose goal 이동 (approach/retract용)."""
        if not self._check_above_bed_floor("joint_space_move", [target_pose]):
            return False
        js = self.get_current_joint_state()
        if js is None:
            return False

        # Position constraint (5mm sphere)
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [0.005]

        bv = BoundingVolume()
        bv.primitives = [prim]
        bp = Pose()
        bp.position = target_pose.position
        bp.orientation.w = 1.0
        bv.primitive_poses = [bp]

        pos_con = PositionConstraint()
        pos_con.header.frame_id = BASE_LINK
        pos_con.link_name = TCP_LINK
        pos_con.constraint_region = bv
        pos_con.weight = 1.0

        ori_con = OrientationConstraint()
        ori_con.header.frame_id = BASE_LINK
        ori_con.link_name = TCP_LINK
        ori_con.orientation = target_pose.orientation
        ori_con.absolute_x_axis_tolerance = 0.05
        ori_con.absolute_y_axis_tolerance = 0.05
        ori_con.absolute_z_axis_tolerance = 0.05
        ori_con.weight = 1.0

        goal_c = Constraints()
        goal_c.position_constraints = [pos_con]
        goal_c.orientation_constraints = [ori_con]

        mpr = MotionPlanRequest()
        mpr.group_name = PLANNING_GROUP
        mpr.num_planning_attempts = 20
        mpr.allowed_planning_time = 10.0
        mpr.max_velocity_scaling_factor = velocity_scale
        mpr.max_acceleration_scaling_factor = velocity_scale
        mpr.start_state.joint_state = js
        mpr.goal_constraints = [goal_c]

        req = GetMotionPlan.Request()
        req.motion_plan_request = mpr

        future = self._plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error("Plan service failed.")
            return False

        resp = future.result().motion_plan_response
        if resp.error_code.val != 1:
            self.get_logger().error(f"Joint-space planning failed: {resp.error_code.val}")
            return False

        traj = resp.trajectory
        self.scale_trajectory_speed(traj.joint_trajectory, velocity_scale)

        self.get_logger().info(
            f"Joint-space path planned ({len(traj.joint_trajectory.points)} pts)."
        )
        return self.execute_trajectory(traj)

    def cartesian_move(self, waypoints, max_step=0.005, velocity_scale=0.5):
        """Cartesian path 계획 + 실행."""
        result = self.compute_cartesian(waypoints, max_step, velocity_scale)
        if result is None:
            self.record_failed_path("cartesian_plan_failed", waypoints)
            return False
        traj, _ = result
        ok = self.execute_trajectory(traj)
        if ok:
            self.record_executed_path("cartesian", waypoints)
        else:
            self.record_failed_path("cartesian_execute_failed", waypoints)
        return ok

    def circle_move_seeded(self, waypoints, velocity_scale=0.5):
        """원형 경로를 seeded IK 해들의 단일 joint trajectory로 실행."""
        if not self._check_above_bed_floor("circle_move_seeded", waypoints):
            return False
        self.publish_path_preview(waypoints)
        self.publish_progress_point(waypoints[0])

        joint_path = self.compute_seeded_ik_joint_path(waypoints)
        if joint_path is None:
            self.record_failed_path("circle_seeded_ik_failed", waypoints)
            return False

        robot_traj = self.build_joint_trajectory_from_path(joint_path, velocity_scale)
        if robot_traj is None:
            self.record_failed_path("circle_seeded_trajectory_failed", waypoints)
            return False

        self.get_logger().info(
            f"Circle seeded IK trajectory planned ({len(joint_path)} IK pts, "
            f"{len(robot_traj.joint_trajectory.points)} trajectory pts)."
        )
        ok = self.execute_trajectory(robot_traj)
        if ok:
            for wp in waypoints:
                self.publish_progress_point(wp)
            self.record_executed_path("circle_seeded_ik_joint_trajectory", waypoints)
        else:
            self.record_failed_path("circle_seeded_execute_failed", waypoints)
        return ok

    def get_current_tcp_pose(self):
        """TF에서 현재 TCP 위치 읽기."""
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            tf = self._tf_buffer.lookup_transform(BASE_LINK, TCP_LINK, rclpy.time.Time())
            p = Pose()
            p.position.x = tf.transform.translation.x
            p.position.y = tf.transform.translation.y
            p.position.z = tf.transform.translation.z
            p.orientation.x = tf.transform.rotation.x
            p.orientation.y = tf.transform.rotation.y
            p.orientation.z = tf.transform.rotation.z
            p.orientation.w = tf.transform.rotation.w
            return p
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def safe_approach(self, target_pose, safe_z, max_step, velocity_scale, prefer_joint=False):
        """
        Z가 safe_z 아래로 절대 내려가지 않는 3단계 Cartesian 접근.
          1. 현재 XY에서 safe_z로 수직 상승
          2. target XY로 수평 이동 (safe_z 유지)
          3. target Z로 수직 하강

        safe_z보다 target_z가 높으면 safe_z를 target_z로 조정.
        """
        cur = self.get_current_tcp_pose()
        if cur is None:
            self.get_logger().error("safe_approach: TCP 위치를 읽을 수 없음.")
            return False

        cur_quat = [cur.orientation.x, cur.orientation.y,
                    cur.orientation.z, cur.orientation.w]
        quat = [target_pose.orientation.x, target_pose.orientation.y,
                target_pose.orientation.z, target_pose.orientation.w]
        tx = target_pose.position.x
        ty = target_pose.position.y
        tz = target_pose.position.z

        # safe_z는 현재 Z, target Z 모두보다 높아야 의미 있음
        floor_z = max(safe_z, tz + 0.005)

        waypoints = []

        # 1. 현재 위치에서 safe_z로 수직 상승 — 현재 자세 유지 (자세 전환 없음)
        if cur.position.z < floor_z - 0.003:
            waypoints.append(make_pose([cur.position.x, cur.position.y, floor_z], cur_quat))

        # 2. target XY로 수평 이동하면서 목표 자세로 전환
        waypoints.append(make_pose([tx, ty, floor_z], quat))

        # 3. target Z로 하강 (이미 floor_z == tz이면 스킵)
        if floor_z - tz > 0.005:
            waypoints.append(make_pose([tx, ty, tz], quat))

        self.get_logger().info(
            f"safe_approach: {len(waypoints)} steps, safe_z={floor_z:.3f}m → target_z={tz:.3f}m"
        )
        if prefer_joint:
            safe_pose = make_pose([tx, ty, floor_z], quat)
            self.get_logger().info("safe_approach: using joint-space move to safe pose first.")
            if not self.joint_space_move(safe_pose, velocity_scale):
                self.get_logger().error("safe_approach joint-space safe pose move failed.")
                return False
            if floor_z - tz <= 0.005:
                return True
            return self.cartesian_move([make_pose([tx, ty, tz], quat)], max_step, velocity_scale)

        if self.cartesian_move_segmented(waypoints, max_step, velocity_scale):
            return True

        self.get_logger().warn("safe_approach Cartesian failed. Falling back to joint-space safe pose move.")
        safe_pose = make_pose([tx, ty, floor_z], quat)
        if not self.joint_space_move(safe_pose, velocity_scale):
            return False
        if floor_z - tz <= 0.005:
            return True
        return self.cartesian_move([make_pose([tx, ty, tz], quat)], max_step, velocity_scale)

    def cartesian_move_segmented(self, waypoints, max_step=0.005, velocity_scale=0.5):
        """긴 waypoint 체인을 짧은 구간으로 나눠 실행."""
        if len(waypoints) < 2:
            return self.cartesian_move(waypoints, max_step, velocity_scale)
        self.publish_path_preview(waypoints)
        self.publish_progress_point(waypoints[0])
        for i in range(len(waypoints) - 1):
            ok = self.cartesian_move(
                [waypoints[i], waypoints[i + 1]], max_step, velocity_scale
            )
            if not ok:
                self.get_logger().error(f"Segmented move failed at segment {i}.")
                return False
            self.publish_progress_point(waypoints[i + 1])
        return True

    def max_reachable_line_len(self, start_pos, axis_u, quat, target_len, max_step=0.005):
        """현재 자세에서 Cartesian으로 가능한 선 길이(0~target_len)를 이분 탐색으로 추정."""
        lo, hi = 0.0, float(target_len)
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            wps = build_line_waypoints(start_pos, axis_u, quat, mid)
            result = self.compute_cartesian(wps, max_step=max_step, velocity_scale=1.0)
            if result is not None:
                lo = mid
            else:
                hi = mid
        return lo


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()

    if os.path.exists(HOME_ZERO_LOCK):
        age = time.time() - os.path.getmtime(HOME_ZERO_LOCK)
        if age < 900.0:
            print(
                "\n[ABORT] Home Zero가 아직 실행 중입니다. "
                "완료 후 Run Stacking을 다시 실행하세요."
            )
            rclpy.shutdown()
            return
        print("[WARN] 오래된 Home Zero lock을 무시합니다.")

    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)

    origin_pos = np.array(cfg["bed"]["origin"]["position"], dtype=float)
    z_offset = float(cfg["motion"].get("z_offset", 0.0))
    origin_quat = cfg["bed"]["origin"]["orientation"]
    width = float(cfg["bed"]["width"])
    height = float(cfg["bed"]["height"])
    physical_width = float(cfg["bed"].get("physical_width", width))
    physical_height = float(cfg["bed"].get("physical_height", height))
    num_layers = cfg["layer"]["num_layers"]
    layer_height = cfg["layer"]["layer_height"]
    line_spacing = cfg["layer"]["line_spacing"]
    approach_height = cfg["motion"]["approach_height"]
    approach_only = bool(cfg["motion"].get("approach_only", False))
    direct_teach_approach = bool(cfg["motion"].get("direct_teach_approach", True))
    teach_move_duration = float(cfg["motion"].get("teach_move_duration", 30.0))
    max_step = cfg["motion"]["cartesian_max_step"]
    velocity_scale = cfg["motion"]["velocity_scale"]
    cartesian_min_fraction = float(cfg["motion"].get("cartesian_min_fraction", 0.99))
    prefer_joint_approach = bool(cfg["motion"].get("prefer_joint_approach", True))
    safe_margin = float(cfg["motion"].get("safe_margin", 0.03))
    min_bed_clearance = float(cfg["motion"].get("min_bed_clearance", 0.0))
    diagonal_lift_deg = float(cfg["motion"].get("diagonal_lift_deg", 0.0))
    pattern = cfg["layer"].get("pattern", "helix")
    helix_points_per_turn = int(cfg["layer"].get("helix_points_per_turn", 72))
    helix_start_angle_deg = float(cfg["layer"].get("helix_start_angle_deg", 45.0))
    helix_radius_scale = float(cfg["layer"].get("helix_radius_scale", 0.45))
    enforce_world_up = bool(cfg["layer"].get("helix_enforce_world_up", True))
    helix_start_clearance = float(cfg["layer"].get("helix_start_clearance", 0.002))
    force_tcp_x_down = bool(cfg.get("tool", {}).get("force_tcp_x_down", True))
    live_plot_enabled = bool(cfg.get("visualization", {}).get("live_plot", True))
    live_plot_rate_hz = float(
        cfg.get("visualization", {}).get("live_plot_rate_hz", 10.0)
    )
    extruder_cfg = cfg.get("extruder", {})
    extruder_enabled = bool(extruder_cfg.get("enabled", False))
    extruder_manual_confirm = bool(
        extruder_cfg.get("prime", {}).get("manual_confirm", True)
    )
    teach_joint_positions_deg = cfg.get("teach", {}).get("joint_positions_deg", [])
    teach_joint_positions = [np.deg2rad(float(x)) for x in teach_joint_positions_deg]

    bed_frame = load_bed_frame(cfg)
    if bed_frame is not None:
        origin_pos = bed_frame["origin"]
        physical_width = bed_frame["physical_width"]
        physical_height = bed_frame["physical_height"]
        bed_normal = bed_frame["normal"]
        axis_u = bed_frame["axis_u"]
        axis_v = bed_frame["axis_v"]
        is_flat_z = str(cfg["bed"].get("calibration", {}).get("z_mode", "")).lower() == "flat_average"
        bed_frame_label = f"4-point {bed_frame['method']}"
        stacking_label = "flat Z / world +Z" if is_flat_z else "calibrated bed normal"
        raster_u_label = "calibrated +U"
        raster_v_label = "calibrated +V"
    else:
        # Fallback: 기존 1점 교시 + 월드축 기준 동작.
        bed_normal = np.array([0.0, 0.0, 1.0])
        axis_u = np.array([1.0, 0.0, 0.0])
        axis_v = np.array([0.0, -1.0, 0.0])
        bed_frame_label = "legacy 1-point/world-axis"
        stacking_label = "world +Z"
        raster_u_label = "world +X"
        raster_v_label = "world -Y"

    origin_pos = origin_pos + z_offset * bed_normal
    safe_z = origin_pos[2] + safe_margin  # 베드 원점 기준 — 재교시 시 자동 추종

    bed_center = (
        origin_pos
        + 0.5 * physical_width * axis_u
        + 0.5 * physical_height * axis_v
    )
    print_origin = (
        bed_center
        - 0.5 * width * axis_u
        - 0.5 * height * axis_v
    )

    if width > physical_width or height > physical_height:
        raise ValueError(
            "Print area must fit inside the physical bed: "
            f"print={width:.3f}x{height:.3f}m, "
            f"bed={physical_width:.3f}x{physical_height:.3f}m"
        )

    if force_tcp_x_down:
        origin_quat = make_tcp_x_down_quat(axis_u, -bed_normal)

    # TCP approach axis (아래 방향): J6 co-rotation용
    R_teach = quat_to_rot(origin_quat)
    tcp_approach = R_teach @ np.array([1.0, 0.0, 0.0])  # TCP local X = approach

    print("=" * 50)
    print("  Planar Layer Motion")
    print("=" * 50)
    print(f"  Bed frame   : {bed_frame_label}")
    print(f"  Bed origin  : {origin_pos.round(4)}")
    print(f"  Bed size    : {physical_width*100:.0f}cm × {physical_height*100:.0f}cm")
    print(f"  Print center: {bed_center.round(4)}")
    print(f"  Stacking    : {stacking_label}")
    print(f"  Raster U    : {raster_u_label} {axis_u.round(4)}")
    print(f"  Raster V    : {raster_v_label} {axis_v.round(4)}")
    print(f"  Bed normal  : {bed_normal.round(4)}")
    print(f"  Area        : {width*100:.0f}cm × {height*100:.0f}cm")
    print(f"  Layers      : {num_layers}  (Δz={layer_height*1000:.1f}mm)")
    print(f"  Pattern     : {pattern}")
    print(f"  Line spacing: {line_spacing*1000:.1f}mm")
    print(f"  Approach    : {approach_height*100:.1f}cm above surface")
    print(f"  Bed floor   : no target below {min_bed_clearance*1000:.1f}mm clearance")
    print(f"  ApproachOnly: {approach_only}")
    print(f"  DirectTeach : {direct_teach_approach} ({teach_move_duration:.1f}s)")
    print(f"  Safe Z      : {safe_z*100:.1f}cm (베드면 +{safe_margin*100:.1f}cm, 이동 중 이 아래로 안 내려감)")
    print(f"  Cartesian   : fraction >= {cartesian_min_fraction*100:.1f}% required")
    print(f"  ApproachMode: {'joint-safe-first' if prefer_joint_approach else 'cartesian-first'}")
    print(f"  Diag lift   : {diagonal_lift_deg:.1f}deg (0=vertical)")
    print(f"  Live plot   : {live_plot_enabled} ({live_plot_rate_hz:.1f}Hz)")
    if extruder_enabled:
        print(
            "  Extruder    : "
            f"{str(extruder_cfg.get('motor', 'A')).upper()} "
            f"interval={int(extruder_cfg.get('interval_us', 1000))}us "
            f"mode={str(extruder_cfg.get('mode', 'ROS')).upper()} "
            f"manual_confirm={extruder_manual_confirm}"
        )
    else:
        print("  Extruder    : disabled")
    print("=" * 50)

    if list(origin_pos) == [0.0, 0.0, 0.0]:
        print("\n[WARN] origin이 (0,0,0) → teach_bed.py 먼저 실행하세요.")
        ans = input("계속? (y/N): ").strip().lower()
        if ans != "y":
            rclpy.shutdown()
            return

    preview_waypoints = []
    if pattern == "helix":
        preview_waypoints = helix_waypoints(
            bed_center,
            axis_u,
            axis_v,
            bed_normal,
            origin_quat,
            radius=max(0.001, min(width, height) * helix_radius_scale),
            total_turns=float(num_layers),
            total_height=max(0.0, (num_layers - 1) * layer_height),
            points_per_turn=helix_points_per_turn,
            start_angle_deg=helix_start_angle_deg,
            enforce_world_up=enforce_world_up,
        )
    elif pattern == "circle":
        radius = max(0.001, min(width, height) * helix_radius_scale)
        for layer_idx in range(num_layers):
            preview_waypoints.extend(
                circle_waypoints(
                    bed_center + layer_idx * layer_height * bed_normal,
                    axis_u,
                    axis_v,
                    origin_quat,
                    radius=radius,
                    points_per_turn=helix_points_per_turn,
                    start_angle_deg=helix_start_angle_deg,
                    approach_axis=tcp_approach,
                )
            )
    else:
        for layer_idx in range(num_layers):
            preview_waypoints.extend(
                raster_waypoints(
                    print_origin,
                    axis_u,
                    axis_v,
                    origin_quat,
                    width,
                    height,
                    line_spacing,
                    layer_idx * layer_height,
                    bed_normal,
                )
            )

    bed_corners = np.array([
        origin_pos,
        origin_pos + physical_width * axis_u,
        origin_pos + physical_width * axis_u + physical_height * axis_v,
        origin_pos + physical_height * axis_v,
        origin_pos,
    ])[:, :2]

    node = PlanarMotionNode(cartesian_min_fraction=cartesian_min_fraction)
    node.configure_bed_floor(origin_pos, bed_normal, min_clearance=min_bed_clearance)
    extruder_runtime = {"started": False, "motor": str(extruder_cfg.get("motor", "A")).upper()}

    def _stop_extruder_at_exit():
        if not extruder_runtime["started"] or not rclpy.ok():
            return
        try:
            node.stop_extruder_motor(extruder_runtime["motor"])
            print(f"[Extruder] exit cleanup: STOP {extruder_runtime['motor']}")
        except Exception as exc:
            print(f"[Extruder] exit cleanup failed: {exc}")

    atexit.register(_stop_extruder_at_exit)

    if live_plot_enabled:
        node.start_live_plot(
            bed_corners,
            preview_waypoints,
            update_rate_hz=live_plot_rate_hz,
        )
    node.start_actual_recording(rate_hz=20.0)

    input("\n▶ Enter → Approach 위치로 이동...")
    approach_pos = origin_pos + approach_height * bed_normal

    # Step 1: Approach
    if direct_teach_approach:
        if len(teach_joint_positions) != 6:
            print("[ABORT] teach.joint_positions_deg 6개가 필요합니다.")
            rclpy.shutdown()
            return
        print("\n[1] Approaching (direct teach-joint trajectory)...")
        ok = node.direct_joint_move(teach_joint_positions, teach_move_duration)
        if not ok:
            print("[ABORT] Direct teach-joint approach 실패.")
            rclpy.shutdown()
            return
        if approach_only:
            print("[완료] Approach-only 진단 모드 완료. direct teach-joint 이동만 실행함.")
            node.stop_actual_recording()
            plot_path = os.path.expanduser("~/planar_layer_executed.png")
            saved_plot = node.save_execution_plot([], plot_path)
            if saved_plot:
                print(f"[시각화] 실행 경로 저장: {saved_plot}")
            actual_csv, command_csv, joint_plot = node.save_joint_tracking_debug(
                os.path.expanduser("~/planar_layer_debug")
            )
            print(f"[디버그] 실제 joint CSV: {actual_csv}")
            print(f"[디버그] 명령 joint CSV: {command_csv}")
            if joint_plot:
                print(f"[디버그] joint 추종 플롯: {joint_plot}")
            rclpy.shutdown()
            return
        print("\n[1] Lifting vertically from current teach pose...")
        cur = node.get_current_tcp_pose()
        if cur is None:
            print("[ABORT] 현재 TCP 위치를 읽을 수 없어 lift 불가.")
            rclpy.shutdown()
            return
        cur_pos = np.array([cur.position.x, cur.position.y, cur.position.z], dtype=float)
        cur_quat = [
            cur.orientation.x,
            cur.orientation.y,
            cur.orientation.z,
            cur.orientation.w,
        ]
        current_clearance = float(np.dot(cur_pos - origin_pos, bed_normal))
        print(f"  current TCP: {cur_pos.round(4)}")
        print(f"  clearance  : {current_clearance*100:.1f}cm above bed origin")
        if current_clearance >= approach_height - 0.002:
            print("  이미 approach 높이 이상 → lift 생략.")
            ok = True
        else:
            lift_pos = cur_pos + (approach_height - current_clearance) * bed_normal
            print(f"  lift target: {lift_pos.round(4)} (+{(approach_height-current_clearance)*100:.1f}cm)")
            lift_pose = make_pose(lift_pos, cur_quat)
            ok = node.cartesian_move([lift_pose], max_step, min(velocity_scale, 0.10))
            if not ok:
                print("  Cartesian lift 실패. 현재 teach pose를 유지하고 계속 진행합니다.")
                ok = True
    else:
        print("\n[1] Approaching (safe Cartesian)...")
        ok = node.safe_approach(
            make_pose(approach_pos, origin_quat),
            safe_z,
            max_step,
            velocity_scale,
            prefer_joint=prefer_joint_approach,
        )
    if not ok:
        print("[ABORT] Approach 실패.")
        rclpy.shutdown()
        return

    runtime_pose = node.get_current_tcp_pose()
    if runtime_pose is not None:
        runtime_quat = [
            runtime_pose.orientation.x,
            runtime_pose.orientation.y,
            runtime_pose.orientation.z,
            runtime_pose.orientation.w,
        ]
        runtime_tcp_approach = quat_to_rot(runtime_quat) @ np.array([1.0, 0.0, 0.0])
        print("  Runtime TCP : 현재 approach 자세를 출력 자세로 사용.")
    else:
        runtime_quat = origin_quat
        runtime_tcp_approach = tcp_approach
        print("  Runtime TCP : 현재 자세 읽기 실패 → 설정 자세 사용.")

    if approach_only:
        print("[완료] Approach-only 진단 모드 완료. 적층/래스터는 실행하지 않음.")
        node.stop_actual_recording()
        plot_path = os.path.expanduser("~/planar_layer_executed.png")
        saved_plot = node.save_execution_plot([], plot_path)
        if saved_plot:
            print(f"[시각화] 실행 경로 저장: {saved_plot}")
        actual_csv, command_csv, joint_plot = node.save_joint_tracking_debug(
            os.path.expanduser("~/planar_layer_debug")
        )
        print(f"[디버그] 실제 joint CSV: {actual_csv}")
        print(f"[디버그] 명령 joint CSV: {command_csv}")
        if joint_plot:
            print(f"[디버그] joint 추종 플롯: {joint_plot}")
        rclpy.shutdown()
        return

    if extruder_enabled:
        try:
            motor = node.start_extruder_prime(extruder_cfg)
            extruder_runtime["started"] = True
            extruder_runtime["motor"] = motor
            print(f"\n[Extruder] START {motor} prime 시작.")
        except Exception as exc:
            print(f"[ABORT] Extruder prime 시작 실패: {exc}")
            node.stop_actual_recording()
            node.stop_live_plot()
            rclpy.shutdown()
            return

        if extruder_manual_confirm:
            input("\n▶ 노즐 끝에 소재가 도착하면 Enter → 적층 시작...")
        else:
            fixed_seconds = float(extruder_cfg.get("prime", {}).get("fixed_seconds", 0.0))
            if fixed_seconds > 0.0:
                print(f"\n[Extruder] fixed prime wait {fixed_seconds:.1f}s...")
                time.sleep(fixed_seconds)
        node.stop_extruder_motor(motor)
        extruder_runtime["started"] = False
        print(f"[Extruder] prime 완료 → STOP {motor}. 출력 시작점 이동 후 다시 START.")
    else:
        input("\n▶ Enter → 적층 시작...")
    node.clear_path_records()

    completed = True
    last_layer_pos = origin_pos + (num_layers - 1) * layer_height * bed_normal
    planned_waypoints = []

    def start_path_extrusion():
        if not extruder_enabled:
            return True
        if extruder_runtime["started"]:
            return True
        try:
            motor = extruder_runtime["motor"]
            node.send_extruder_command(f"START {motor}")
            extruder_runtime["started"] = True
            print(f"[Extruder] START {motor} 출력 경로 시작.")
            return True
        except Exception as exc:
            print(f"[ABORT] Extruder START 실패: {exc}")
            return False

    def stop_path_extrusion(reason=""):
        if not extruder_enabled or not extruder_runtime["started"]:
            return
        try:
            motor = extruder_runtime["motor"]
            node.stop_extruder_motor(motor)
            extruder_runtime["started"] = False
            suffix = f" ({reason})" if reason else ""
            print(f"[Extruder] STOP {motor}{suffix}")
        except Exception as exc:
            print(f"[Extruder] STOP 실패: {exc}")

    if pattern == "helix":
        total_height = max(0.0, (num_layers - 1) * layer_height)
        base_radius = max(0.001, min(width, height) * helix_radius_scale)
        center = bed_center

        radius_scales = [1.0, 0.75, 0.55, 0.40]
        helix_ok = False
        for idx, r_scale in enumerate(radius_scales):
            radius = base_radius * r_scale
            helix_wps = helix_waypoints(
                center, axis_u, axis_v, bed_normal, runtime_quat,
                radius=radius,
                total_turns=float(num_layers),
                total_height=total_height,
                points_per_turn=helix_points_per_turn,
                start_angle_deg=helix_start_angle_deg,
                enforce_world_up=enforce_world_up,
            )
            planned_waypoints.extend(helix_wps)

            start_wp = helix_wps[0]
            start_pos = np.array([start_wp.position.x, start_wp.position.y, start_wp.position.z])
            start_pos = start_pos + helix_start_clearance * bed_normal

            print(f"\n[Helix] 시작점 상부로 Cartesian 이동... (trial {idx+1}, r={radius*1000:.1f}mm)")
            ok = node.cartesian_move(
                [make_pose(start_pos + approach_height * bed_normal, runtime_quat)],
                max_step,
                velocity_scale,
            )
            if not ok:
                print("  [WARN] 시작 접근 실패. 반경 축소 재시도.")
                continue

            ok = node.cartesian_move([make_pose(start_pos, runtime_quat)], max_step, velocity_scale)
            if not ok:
                print("  [WARN] Cartesian 시작 하강 실패. joint-space fallback...")
                ok = node.joint_space_move(make_pose(start_pos, runtime_quat), velocity_scale)
                if not ok:
                    print("  [WARN] 시작 하강 실패. 반경 축소 재시도.")
                    continue

            print(f"[Helix] 원형 적층 실행... (r={radius*1000:.1f}mm, turns={num_layers})")
            if not start_path_extrusion():
                completed = False
                break
            ok = node.cartesian_move_segmented(helix_wps, max_step, velocity_scale)
            if ok:
                last_wp = helix_wps[-1].position
                last_layer_pos = np.array([last_wp.x, last_wp.y, last_wp.z])
                helix_ok = True
                break
            stop_path_extrusion("Helix retry")
            print("  [WARN] Helix 실패. 반경 축소 재시도.")

        if not helix_ok:
            print("  [ABORT] Helix 적층 실패.")
            completed = False
    elif pattern == "circle":
        base_radius = max(0.001, min(width, height) * helix_radius_scale)
        radius_scales = [1.0, 0.75, 0.55, 0.40]
        start_center = bed_center

        circle_ok = False
        for idx, r_scale in enumerate(radius_scales):
            radius = base_radius * r_scale
            print(f"\n[Circle] trial {idx+1}, r={radius*1000:.1f}mm")
            layer_ok = True

            for layer_idx in range(num_layers):
                z_offset = layer_idx * layer_height
                layer_center = start_center + z_offset * bed_normal
                wps = circle_waypoints(
                    layer_center, axis_u, axis_v, runtime_quat,
                    radius=radius,
                    points_per_turn=helix_points_per_turn,
                    start_angle_deg=helix_start_angle_deg,
                    approach_axis=None,
                )
                planned_waypoints.extend(wps)

                if layer_idx == 0:
                    start_wp = wps[0]
                    start_pos = np.array([start_wp.position.x, start_wp.position.y, start_wp.position.z])
                    start_pos = start_pos + helix_start_clearance * bed_normal
                    start_above_pose = make_pose(start_pos + approach_height * bed_normal, runtime_quat)
                    start_pose = make_pose(start_pos, runtime_quat)
                    print("  → 시작점 상부로 Cartesian 이동...")
                    ok = node.cartesian_move(
                        [start_above_pose],
                        max_step,
                        velocity_scale,
                    )
                    if not ok:
                        print("  [WARN] Cartesian 시작 접근 실패. seeded IK fallback...")
                        ok = node.seeded_ik_move(
                            start_above_pose,
                            min(velocity_scale, 0.12),
                            label="circle_start_above",
                        )
                        if not ok:
                            print("  [WARN] 시작 접근 실패.")
                            layer_ok = False
                            break
                    ok = node.cartesian_move([start_pose], max_step, velocity_scale)
                    if not ok:
                        print("  [WARN] Cartesian 시작 하강 실패. seeded IK fallback...")
                        ok = node.seeded_ik_move(
                            start_pose,
                            min(velocity_scale, 0.12),
                            label="circle_start_surface",
                        )
                        if not ok:
                            print("  [WARN] 시작 하강 실패.")
                            layer_ok = False
                            break
                else:
                    # 다음 레이어 시작점으로 짧게 상승 이동
                    next_start = wps[0]
                    ok = node.cartesian_move([next_start], max_step, velocity_scale)
                    if not ok:
                        print("  [WARN] Cartesian 레이어 전환 실패. joint-space fallback...")
                        ok = node.joint_space_move(next_start, velocity_scale)
                        if not ok:
                            print("  [WARN] 레이어 전환 실패.")
                            layer_ok = False
                            break

                print(f"  → Circle layer {layer_idx+1}/{num_layers}  (r={radius*100:.1f}cm, {len(wps)} pts)")
                if not start_path_extrusion():
                    layer_ok = False
                    break
                print("  → Cartesian trajectory로 원 실행")
                ok = node.cartesian_move(wps, max_step, min(velocity_scale, 0.12))
                if not ok:
                    print("  [WARN] Cartesian 원 실행 실패. seeded IK fallback...")
                    ok = node.circle_move_seeded(wps, min(velocity_scale, 0.12))
                    if not ok:
                        stop_path_extrusion("Circle retry")
                        layer_ok = False
                        break
                last_wp = wps[-1].position
                last_layer_pos = np.array([last_wp.x, last_wp.y, last_wp.z])

            if layer_ok:
                circle_ok = True
                break
            print("  [WARN] Circle 실패. 반경 축소 재시도.")

        if not circle_ok:
            print("  [ABORT] Circle 적층 실패.")
            completed = False
    else:
        for layer_idx in range(num_layers):
            z_offset = layer_idx * layer_height
            print(f"\n[Layer {layer_idx+1}/{num_layers}]  z_offset={z_offset*1000:.1f}mm")

            # 레이어 표면 원점
            layer_start = print_origin + z_offset * bed_normal

            # 하강
            print("  → 하강...")
            ok = node.cartesian_move([make_pose(layer_start, runtime_quat)], max_step, velocity_scale)
            if not ok:
                print("  [WARN] Cartesian 하강 실패. joint-space fallback...")
                ok = node.joint_space_move(make_pose(layer_start, runtime_quat), velocity_scale)
                if not ok:
                    print("  [ABORT] 하강 실패.")
                    break

            # 래스터
            print("  → 래스터 스캔...")
            if not start_path_extrusion():
                completed = False
                break
            max_u = node.max_reachable_line_len(layer_start, axis_u, runtime_quat, width, max_step=max_step)
            if max_u < 0.5 * width:
                print(f"  [ABORT] 시작 선 reachable 길이 부족 ({max_u*1000:.1f}mm / {width*1000:.1f}mm).")
                completed = False
                break
            if max_u < width:
                print(f"  [WARN] width 자동 축소: {width*1000:.1f}mm -> {max_u*1000:.1f}mm")
            wps = raster_waypoints(
                print_origin, axis_u, axis_v, runtime_quat, max_u, height, line_spacing, z_offset, bed_normal
            )
            planned_waypoints.extend(wps)
            ok = node.cartesian_move_segmented(wps, max_step, velocity_scale)
            if not ok:
                stop_path_extrusion("Raster failed")
                print("  [ABORT] 래스터 실패.")
                completed = False
                break
            last_wp = wps[-1].position
            last_layer_pos = np.array([last_wp.x, last_wp.y, last_wp.z])

            # 다음 레이어 전 상승
            if layer_idx < num_layers - 1:
                print("  → 상승...")
                last_wp = wps[-1]
                last_pos = np.array([
                    last_wp.position.x,
                    last_wp.position.y,
                    last_wp.position.z,
                ])
                if diagonal_lift_deg > 0.0:
                    lift_pos = diagonal_lift_target(
                        last_pos, bed_normal, axis_u, approach_height, diagonal_lift_deg
                    )
                else:
                    lift_pos = last_pos + approach_height * bed_normal
                ok = node.cartesian_move([make_pose(lift_pos, runtime_quat)], max_step, velocity_scale)
                if not ok:
                    print("  [ABORT] 상승 실패.")
                    completed = False
                    break

    if extruder_runtime["started"] and bool(extruder_cfg.get("stop_on_finish", True)):
        node.stop_extruder_motor(extruder_runtime["motor"])
        print(f"[Extruder] STOP {extruder_runtime['motor']} 전송.")
        extruder_runtime["started"] = False

    # 완료 후 상승
    print("\n[완료] 안전 높이로 복귀...")
    safe_pos = last_layer_pos + approach_height * bed_normal
    ok = node.cartesian_move([make_pose(safe_pos, runtime_quat)], max_step, velocity_scale)
    if not ok:
        print("  [WARN] Cartesian 복귀 실패. Joint-space fallback...")
        ok = node.joint_space_move(make_pose(safe_pos, runtime_quat), velocity_scale)
        if not ok:
            print("  [WARN] Joint-space 복귀 실패. seeded IK fallback...")
            node.seeded_ik_move(
                make_pose(safe_pos, runtime_quat),
                min(velocity_scale, 0.12),
                label="final_safe_return",
            )

    if completed:
        print("[완료] Planar layer motion 완료.")
    else:
        print("[중단] Planar layer motion 중간 중단.")
    js = node.get_current_joint_state()
    if js is not None:
        pos_by_name = dict(zip(js.name, js.position))
        if all(name in pos_by_name for name in JOINT_NAMES):
            node._publish_hold_current([float(pos_by_name[name]) for name in JOINT_NAMES])
            print("[완료] 현재 자세 hold 명령 전송.")
    node.stop_actual_recording()
    node.stop_live_plot()
    plot_path = os.path.expanduser("~/planar_layer_executed.png")
    saved_plot = node.save_execution_plot(planned_waypoints, plot_path)
    if saved_plot:
        print(f"[시각화] 실행 경로 저장: {saved_plot}")
    actual_csv, command_csv, joint_plot = node.save_joint_tracking_debug(
        os.path.expanduser("~/planar_layer_debug")
    )
    print(f"[디버그] 실제 joint CSV: {actual_csv}")
    print(f"[디버그] 명령 joint CSV: {command_csv}")
    if joint_plot:
        print(f"[디버그] joint 추종 플롯: {joint_plot}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
