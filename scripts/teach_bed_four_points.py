#!/usr/bin/env python3
"""
Four-point bed teaching script.

Usage:
  1. Start Robot Launch so /tf publishes base_link -> tcp.
  2. Move the nozzle tip into each bed groove in this order:
       front_left -> front_right -> back_left -> back_right
  3. Press Enter at each prompt.

The script stores a calibrated bed frame in config/bed_config.yaml. Motion code
uses this frame when bed.calibration.enabled is true.
"""

import math
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import tf2_ros
import yaml


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")
BASE_LINK = "base_link"
TCP_LINK = "tcp"

POINT_ORDER = [
    ("front_left", "좌앞 홈"),
    ("front_right", "우앞 홈"),
    ("back_left", "좌뒤 홈"),
    ("back_right", "우뒤 홈"),
]


def normalize(v, label):
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError(f"{label} 벡터 길이가 너무 작습니다.")
    return v / n


def angle_deg(a, b):
    dot = float(np.clip(np.dot(normalize(a, "a"), normalize(b, "b")), -1.0, 1.0))
    return math.degrees(math.acos(dot))


def rot_to_quat(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        return [
            (R[2, 1] - R[1, 2]) * s,
            (R[0, 2] - R[2, 0]) * s,
            (R[1, 0] - R[0, 1]) * s,
            0.25 / s,
        ]
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]


def read_tcp(tf_buffer, executor):
    for _ in range(10):
        executor.spin_once(timeout_sec=0.1)

    transform = tf_buffer.lookup_transform(BASE_LINK, TCP_LINK, rclpy.time.Time())
    t = transform.transform.translation
    r = transform.transform.rotation
    pos = np.array([t.x, t.y, t.z], dtype=float)
    quat = [float(r.x), float(r.y), float(r.z), float(r.w)]
    return pos, quat


def compute_calibration(points, flatten_z=False):
    p00 = np.array(points["front_left"], dtype=float)
    p10 = np.array(points["front_right"], dtype=float)
    p01 = np.array(points["back_left"], dtype=float)
    p11 = np.array(points["back_right"], dtype=float)

    measured_points = {
        "front_left": p00.copy(),
        "front_right": p10.copy(),
        "back_left": p01.copy(),
        "back_right": p11.copy(),
    }
    measured_z_values = np.array([p00[2], p10[2], p01[2], p11[2]], dtype=float)
    measured_z_range_m = float(measured_z_values.max() - measured_z_values.min())
    z_reference_m = float(measured_z_values.mean())

    if flatten_z:
        for p in (p00, p10, p01, p11):
            p[2] = z_reference_m

    front_edge = p10 - p00
    back_edge = p11 - p01
    left_edge = p01 - p00
    right_edge = p11 - p10

    axis_u = normalize((front_edge + back_edge) * 0.5, "axis_u")
    v_raw = (left_edge + right_edge) * 0.5
    v_raw = v_raw - np.dot(v_raw, axis_u) * axis_u
    axis_v = normalize(v_raw, "axis_v")
    normal = normalize(np.cross(axis_u, axis_v), "normal")
    if normal[2] < 0.0:
        normal = -normal
    if flatten_z:
        normal = np.array([0.0, 0.0, 1.0])

    physical_width = 0.5 * (np.linalg.norm(front_edge) + np.linalg.norm(back_edge))
    physical_height = 0.5 * (np.linalg.norm(left_edge) + np.linalg.norm(right_edge))

    all_points = np.vstack([p00, p10, p01, p11])
    centroid = all_points.mean(axis=0)
    plane_errors = np.dot(all_points - centroid, normal)
    plane_error_m = float(np.max(np.abs(plane_errors)))

    front_back_width_error = abs(np.linalg.norm(front_edge) - np.linalg.norm(back_edge))
    left_right_height_error = abs(np.linalg.norm(left_edge) - np.linalg.norm(right_edge))
    orthogonality_error_deg = abs(90.0 - angle_deg(axis_u, axis_v))

    # Keep axis_v as taught. Use a right-handed V only for the optional frame quaternion.
    frame_v = axis_v if np.dot(np.cross(axis_u, axis_v), normal) > 0.0 else -axis_v
    frame_quat = rot_to_quat(np.column_stack([axis_u, frame_v, normal]))

    return {
        "origin": p00,
        "measured_points": measured_points,
        "measured_z_range_m": measured_z_range_m,
        "z_reference_m": z_reference_m,
        "z_mode": "flat_average" if flatten_z else "measured_plane",
        "axis_u": axis_u,
        "axis_v": axis_v,
        "normal": normal,
        "physical_width": float(physical_width),
        "physical_height": float(physical_height),
        "plane_error_m": plane_error_m,
        "front_back_width_error_m": float(front_back_width_error),
        "left_right_height_error_m": float(left_right_height_error),
        "orthogonality_error_deg": float(orthogonality_error_deg),
        "frame_quat": frame_quat,
    }


def round_list(values, digits=6):
    return [round(float(v), digits) for v in values]


def main():
    rclpy.init()
    node = Node("teach_bed_four_points")
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    print("=" * 56)
    print("  Teach Bed 4-Point")
    print("=" * 56)
    print("  순서: 좌앞 -> 우앞 -> 좌뒤 -> 우뒤")
    print("  각 홈에 토출구/노즐 끝을 살짝 넣고 Enter를 누르세요.")
    print("  이 스크립트는 로봇을 움직이지 않고 현재 TCP 좌표만 저장합니다.")
    print("=" * 56)

    input("\nTF 수신 준비 후 Enter...")
    for _ in range(20):
        executor.spin_once(timeout_sec=0.1)

    points = {}
    first_quat = None
    try:
        for key, label in POINT_ORDER:
            input(f"\n[{label}]에 노즐을 넣은 뒤 Enter...")
            pos, quat = read_tcp(tf_buffer, executor)
            points[key] = pos
            if first_quat is None:
                first_quat = quat
            print(f"  저장: x={pos[0]:.6f}  y={pos[1]:.6f}  z={pos[2]:.6f}")

        cal = compute_calibration(points)
    except Exception as exc:
        print(f"\n[ERROR] 4점 교시 실패: {exc}")
        rclpy.shutdown()
        return

    print("\n계산 결과")
    print(f"  origin      : {round_list(cal['origin'])}")
    print(f"  axis_u      : {round_list(cal['axis_u'])}")
    print(f"  axis_v      : {round_list(cal['axis_v'])}")
    print(f"  normal      : {round_list(cal['normal'])}")
    print(f"  bed size    : {cal['physical_width'] * 1000:.1f}mm x {cal['physical_height'] * 1000:.1f}mm")
    print(f"  plane error : {cal['plane_error_m'] * 1000:.3f}mm")
    print(f"  width diff  : {cal['front_back_width_error_m'] * 1000:.3f}mm")
    print(f"  height diff : {cal['left_right_height_error_m'] * 1000:.3f}mm")
    print(f"  square err  : {cal['orthogonality_error_deg']:.3f}deg")
    print(f"  z range     : {cal['measured_z_range_m'] * 1000:.3f}mm")

    flatten_z = False
    if cal["measured_z_range_m"] > 0.001:
        print("\n[WARN] 네 점의 월드 Z 차이가 1mm보다 큽니다.")
        print("       베드가 수평이라면 손교시 누름/홈 깊이/측정 오차일 수 있습니다.")
    ans_flat = input(
        "\n베드를 수평으로 간주하고 Z를 4점 평균값으로 강제할까요? "
        "(Y/n): "
    ).strip().lower()
    if ans_flat in {"", "y", "yes"}:
        flatten_z = True
        cal = compute_calibration(points, flatten_z=True)
        print("\n[flat z] Z 평균 강제 적용")
        print(f"  z reference : {cal['z_reference_m']:.6f}m")
        print(f"  origin      : {round_list(cal['origin'])}")
        print(f"  axis_u      : {round_list(cal['axis_u'])}")
        print(f"  axis_v      : {round_list(cal['axis_v'])}")
        print(f"  normal      : {round_list(cal['normal'])}")

    ans = input("\n이 4점 교시값을 bed_config.yaml에 저장할까요? (y/N): ").strip().lower()
    if ans != "y":
        print("취소됨.")
        rclpy.shutdown()
        return

    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)

    bed = config.setdefault("bed", {})
    origin = bed.setdefault("origin", {})
    origin["position"] = round_list(cal["origin"])
    origin["orientation"] = round_list(first_quat)
    bed["physical_width"] = round(cal["physical_width"], 6)
    bed["physical_height"] = round(cal["physical_height"], 6)
    bed["stacking"] = "world_z" if flatten_z else "calibrated_normal"
    bed["calibration"] = {
        "enabled": True,
        "method": "four_point_flat_z" if flatten_z else "four_point",
        "z_mode": cal["z_mode"],
        "z_reference_m": round(cal["z_reference_m"], 6),
        "measured_z_range_m": round(cal["measured_z_range_m"], 6),
        "frame": BASE_LINK,
        "tcp_link": TCP_LINK,
        "updated_at": datetime.now().isoformat(timespec="seconds"),
        "point_order": [key for key, _ in POINT_ORDER],
        "points": {key: round_list(value) for key, value in points.items()},
        "measured_points": {
            key: round_list(value) for key, value in cal["measured_points"].items()
        },
        "origin": round_list(cal["origin"]),
        "axis_u": round_list(cal["axis_u"]),
        "axis_v": round_list(cal["axis_v"]),
        "normal": round_list(cal["normal"]),
        "frame_quat": round_list(cal["frame_quat"]),
        "physical_width": round(cal["physical_width"], 6),
        "physical_height": round(cal["physical_height"], 6),
        "plane_error_m": round(cal["plane_error_m"], 6),
        "front_back_width_error_m": round(cal["front_back_width_error_m"], 6),
        "left_right_height_error_m": round(cal["left_right_height_error_m"], 6),
        "orthogonality_error_deg": round(cal["orthogonality_error_deg"], 6),
    }

    with open(CONFIG_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

    print(f"\n저장 완료 -> {CONFIG_PATH}")
    print("다음 Run Stacking부터 4점 교시 좌표계가 적용됩니다.")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
