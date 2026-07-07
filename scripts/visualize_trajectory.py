#!/usr/bin/env python3
"""
Trajectory visualizer — ROS2 없이 실행 가능.
bed_config.yaml을 읽어 계획된 경로를 3D 플롯으로 보여줌.

Usage:
  python3 scripts/visualize_trajectory.py
"""

import os
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Line3DCollection

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")


# ── Math ──────────────────────────────────────────────────────────────────────

def quat_to_rot(q):
    qx, qy, qz, qw = q
    return np.array([
        [1-2*(qy**2+qz**2),  2*(qx*qy-qz*qw),  2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),  1-2*(qx**2+qz**2),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),  2*(qy*qz+qx*qw),  1-2*(qx**2+qy**2)],
    ])


# ── Waypoint generators (bed_config.yaml 설정 그대로) ─────────────────────────

def gen_circle(center, axis_u, axis_v, radius, pts, start_deg=0.0):
    start = np.deg2rad(start_deg)
    pts = max(64, pts)
    return np.array([
        center + radius * np.cos(start + 2*np.pi*i/pts) * axis_u
               + radius * np.sin(start + 2*np.pi*i/pts) * axis_v
        for i in range(pts + 1)
    ])


def gen_helix(center, axis_u, axis_v, normal, radius,
              total_turns, total_height, pts_per_turn, start_deg=0.0,
              enforce_world_up=False):
    pts_per_turn = max(64, pts_per_turn)
    total_pts = int(pts_per_turn * total_turns)
    start = np.deg2rad(start_deg)
    world_z = np.array([0., 0., 1.])
    denom = float(np.dot(normal, world_z))
    prev_wz = None
    path = []
    for i in range(total_pts + 1):
        t = i / float(total_pts)
        th = start + 2*np.pi*total_turns*t
        p = (center
             + radius * np.cos(th) * axis_u
             + radius * np.sin(th) * axis_v
             + total_height * t * normal)
        if enforce_world_up and prev_wz is not None and denom > 1e-6:
            wz = float(np.dot(p, world_z))
            if wz < prev_wz:
                dz = (prev_wz - wz) + 1e-5
                p = p + (dz / denom) * normal
                wz = float(np.dot(p, world_z))
            prev_wz = wz
        else:
            prev_wz = float(np.dot(p, world_z))
        path.append(p)
    return np.array(path)


def gen_raster(origin, axis_u, axis_v, width, height, spacing, z_offset, normal):
    layer_origin = origin + z_offset * normal
    pts = []
    v, idx = 0.0, 0
    while v <= height + 1e-6:
        u_vals = [0.0, width] if idx % 2 == 0 else [width, 0.0]
        for u in u_vals:
            pts.append(layer_origin + u * axis_u + v * axis_v)
        v = round(v + spacing, 6)
        idx += 1
    return np.array(pts)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f)

    bed   = cfg["bed"]
    layer = cfg["layer"]
    mot   = cfg["motion"]

    origin_pos  = np.array(bed["origin"]["position"])
    origin_quat = bed["origin"]["orientation"]          # [qx, qy, qz, qw]
    width       = float(bed["width"])
    height      = float(bed["height"])
    stacking    = bed.get("stacking", "bed_z")

    pattern         = layer.get("pattern", "raster")
    num_layers      = int(layer["num_layers"])
    layer_height    = float(layer["layer_height"])
    line_spacing    = float(layer["line_spacing"])
    radius_scale    = float(layer.get("helix_radius_scale", 0.5))
    pts_per_turn    = int(layer.get("helix_points_per_turn", 72))
    start_deg       = float(layer.get("helix_start_angle_deg", 0.0))
    enforce_up      = bool(layer.get("helix_enforce_world_up", False))

    z_offset        = float(mot.get("z_offset", 0.0))
    approach_height = float(mot.get("approach_height", 0.05))

    R       = quat_to_rot(origin_quat)
    axis_u  = R[:, 0]   # bed X
    axis_v  = R[:, 1]   # bed Y
    bed_z   = R[:, 2]   # bed normal

    if stacking == "world_z":
        normal_vec = np.array([0., 0., 1.])
    else:
        normal_vec = bed_z

    radius = min(width, height) * radius_scale
    center = origin_pos + z_offset * normal_vec

    # ── 경로 생성 ─────────────────────────────────────────────────────────────
    all_layers = []      # list of np.array (N,3)
    approach_pts = []

    if pattern == "helix":
        total_height = num_layers * layer_height
        path = gen_helix(
            center, axis_u, axis_v, normal_vec,
            radius, num_layers, total_height,
            pts_per_turn, start_deg, enforce_up
        )
        all_layers.append(path)
        approach_pts = [center + approach_height * normal_vec, path[0]]

    elif pattern == "circle":
        for li in range(num_layers):
            z_off = z_offset + li * layer_height
            c = origin_pos + z_off * normal_vec
            path = gen_circle(c, axis_u, axis_v, radius, pts_per_turn, start_deg)
            all_layers.append(path)
        approach_pts = [
            origin_pos + (z_offset + approach_height) * normal_vec,
            all_layers[0][0]
        ]

    else:  # raster
        for li in range(num_layers):
            z_off = z_offset + li * layer_height
            path = gen_raster(origin_pos, axis_u, axis_v,
                              width, height, line_spacing, z_off, normal_vec)
            all_layers.append(path)
        approach_pts = [
            origin_pos + (z_offset + approach_height) * normal_vec,
            all_layers[0][0]
        ]

    # ── 플롯 ─────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(11, 8), facecolor="#0f0f1a")
    ax  = fig.add_subplot(111, projection="3d", facecolor="#0f0f1a")

    # 컬러맵: 레이어별 색상
    cmap   = cm.get_cmap("plasma", max(num_layers, 1))
    colors = [cmap(i / max(num_layers - 1, 1)) for i in range(num_layers)]

    # approach 경로 (점선)
    if len(approach_pts) >= 2:
        ap = np.array(approach_pts)
        ax.plot(ap[:, 0], ap[:, 1], ap[:, 2],
                "--", color="#888888", lw=1.2, alpha=0.7, label="approach")

    # 레이어별 경로
    for li, (path, color) in enumerate(zip(all_layers, colors)):
        ax.plot(path[:, 0], path[:, 1], path[:, 2],
                color=color, lw=2.0, alpha=0.95,
                label=f"Layer {li+1}  z={z_offset + li*layer_height*1000:.1f}mm")
        # 시작점 마커
        ax.scatter(*path[0], color=color, s=40, zorder=5)
        # 끝점 마커
        ax.scatter(*path[-1], color=color, s=20, marker="x", zorder=5)

    # 베드 origin 마커
    ax.scatter(*origin_pos, color="white", s=80, zorder=10, label="bed origin")

    # 축 구성
    all_pts = np.vstack(all_layers)
    pad = max(width, height) * 0.5
    xm, ym, zm = all_pts.mean(0)
    rng = max(all_pts.ptp(0).max(), 0.02) / 2 + pad

    ax.set_xlim(xm - rng, xm + rng)
    ax.set_ylim(ym - rng, ym + rng)
    ax.set_zlim(zm - rng * 0.5, zm + rng * 1.2)

    ax.set_xlabel("X (m)", color="white", labelpad=6)
    ax.set_ylabel("Y (m)", color="white", labelpad=6)
    ax.set_zlabel("Z (m)", color="white", labelpad=6)
    ax.tick_params(colors="white")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#333355")
    ax.grid(True, color="#222244", linewidth=0.5)

    pattern_label = {"circle": "Circle", "helix": "Helix", "raster": "Raster"}.get(pattern, pattern)
    fig.suptitle(
        f"Planned Trajectory  ·  {pattern_label}  ·  {num_layers} layers  ·  r={radius*100:.1f} cm",
        color="white", fontsize=14, y=0.97
    )
    leg = ax.legend(loc="upper left", fontsize=8, framealpha=0.3,
                    labelcolor="white", facecolor="#1a1a2e")

    plt.tight_layout()

    # 저장
    out_path = os.path.join(os.path.dirname(__file__), "../config/trajectory_preview.png")
    plt.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[OK] 저장됨: {out_path}")

    plt.show()


if __name__ == "__main__":
    main()
