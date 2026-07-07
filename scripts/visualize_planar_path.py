#!/usr/bin/env python3
import os
import argparse
import yaml
import numpy as np

import matplotlib.pyplot as plt

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")


def quat_to_rot(q):
    qx, qy, qz, qw = q
    return np.array([
        [1 - 2*(qy**2 + qz**2),  2*(qx*qy - qz*qw),  2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw),  1 - 2*(qx**2 + qz**2),  2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),  2*(qy*qz + qx*qw),  1 - 2*(qx**2 + qy**2)],
    ])


def pick_normal_from_axis(R, axis_name, sign):
    axis_map = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    if axis_name not in axis_map:
        raise ValueError("bed.normal_axis must be one of x/y/z")
    s = -1.0 if float(sign) < 0.0 else 1.0
    v = s * (R @ axis_map[axis_name])
    return v / np.linalg.norm(v)


def build_axes(cfg):
    origin_pos = np.array(cfg["bed"]["origin"]["position"], dtype=float)
    z_offset = float(cfg["motion"].get("z_offset", 0.0))
    origin_pos = origin_pos + np.array([0.0, 0.0, z_offset])

    # Keep this in sync with planar_layer_motion.py:
    # stacking/raster axes are fixed in world coordinates.
    u = np.array([1.0, 0.0, 0.0])
    v = np.array([0.0, 1.0, 0.0])
    n = np.array([0.0, 0.0, 1.0])

    # Current tool convention in planar_layer_motion.py when force_tcp_x_down=true.
    tcp_x = np.array([0.0, 0.0, -1.0])
    tcp_y = np.array([1.0, 0.0, 0.0])
    tcp_z = np.array([0.0, -1.0, 0.0])

    return {
        "origin": origin_pos,
        "tcp_x": tcp_x,
        "tcp_y": tcp_y,
        "tcp_z": tcp_z,
        "u": u,
        "v": v,
        "n": n,
        "u_name": "world_x",
        "v_name": "world_y",
        "normal_axis": "world_z",
        "normal_sign": 1.0,
    }


def raster_points(origin, u, v, n, width, height, line_spacing, num_layers, layer_height):
    pts = []
    for layer_idx in range(num_layers):
        z = layer_idx * layer_height
        layer_origin = origin + z * n

        vv = 0.0
        line_idx = 0
        while vv <= height + 1e-9:
            u_vals = [0.0, width] if line_idx % 2 == 0 else [width, 0.0]
            for uu in u_vals:
                pts.append(layer_origin + uu * u + vv * v)
            vv = round(vv + line_spacing, 6)
            line_idx += 1
    return np.array(pts)


def helix_points(origin, u, v, n, width, height, num_layers, layer_height, radius_scale, ppr, start_angle_deg):
    center = origin + 0.5 * width * u + 0.5 * height * v
    turns = float(num_layers)
    total_height = max(0.0, (num_layers - 1) * layer_height)
    radius = max(0.001, min(width, height) * float(radius_scale))

    ppr = max(16, int(ppr))
    total_pts = max(1, int(turns * ppr))
    start = np.deg2rad(float(start_angle_deg))

    pts = []
    for i in range(total_pts + 1):
        t = i / float(total_pts)
        th = start + 2.0 * np.pi * turns * t
        p = center + radius * np.cos(th) * u + radius * np.sin(th) * v + total_height * t * n
        pts.append(p)
    return np.array(pts), radius


def circle_points(origin, u, v, n, width, height, num_layers, layer_height, radius_scale, ppr, start_angle_deg):
    # Keep this in sync with planar_layer_motion.py:
    # for circle mode, the taught origin is the circle center.
    center0 = origin
    radius = max(0.001, min(width, height) * float(radius_scale))
    ppr = max(24, int(ppr))
    start = np.deg2rad(float(start_angle_deg))

    pts = []
    for layer_idx in range(num_layers):
        c = center0 + layer_idx * layer_height * n
        for i in range(ppr + 1):
            th = start + 2.0 * np.pi * (i / float(ppr))
            pts.append(c + radius * np.cos(th) * u + radius * np.sin(th) * v)
    return np.array(pts), radius


def equal_axes_3d(ax, xyz):
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    center = (mins + maxs) / 2.0
    radius = np.max(maxs - mins) * 0.6 + 1e-3
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def main():
    parser = argparse.ArgumentParser(description="Visualize planar/helix path from bed_config.yaml")
    parser.add_argument("--config", default=CONFIG_PATH)
    parser.add_argument("--save", default="", help="save figure path (png)")
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)

    axes = build_axes(cfg)
    origin = axes["origin"]
    u = axes["u"]
    v = axes["v"]
    n = axes["n"]

    width = float(cfg["bed"]["width"])
    height = float(cfg["bed"]["height"])
    num_layers = int(cfg["layer"]["num_layers"])
    layer_height = float(cfg["layer"]["layer_height"])
    pattern = str(cfg["layer"].get("pattern", "helix")).lower()

    if pattern == "helix":
        pts, radius = helix_points(
            origin, u, v, n,
            width, height, num_layers, layer_height,
            cfg["layer"].get("helix_radius_scale", 0.45),
            cfg["layer"].get("helix_points_per_turn", 72),
            cfg["layer"].get("helix_start_angle_deg", 45.0),
        )
        subtitle = f"helix radius={radius*1000:.1f}mm"
    elif pattern == "circle":
        pts, radius = circle_points(
            origin, u, v, n,
            width, height, num_layers, layer_height,
            cfg["layer"].get("helix_radius_scale", 0.45),
            cfg["layer"].get("helix_points_per_turn", 72),
            cfg["layer"].get("helix_start_angle_deg", 45.0),
        )
        subtitle = f"circle radius={radius*1000:.1f}mm"
    else:
        pts = raster_points(
            origin, u, v, n,
            width, height, float(cfg["layer"]["line_spacing"]),
            num_layers, layer_height,
        )
        subtitle = "raster"

    fig = plt.figure(figsize=(13, 6))
    ax3 = fig.add_subplot(1, 2, 1, projection="3d")
    ax2 = fig.add_subplot(1, 2, 2)

    ax3.plot(pts[:, 0], pts[:, 1], pts[:, 2], linewidth=1.4)
    ax3.scatter([pts[0, 0]], [pts[0, 1]], [pts[0, 2]], c="green", s=40)
    ax3.scatter([pts[-1, 0]], [pts[-1, 1]], [pts[-1, 2]], c="red", s=40)

    scale = min(width, height) * 0.35
    ax3.quiver(*origin, *(u * scale), color="tab:blue")
    ax3.quiver(*origin, *(v * scale), color="tab:orange")
    ax3.quiver(*origin, *(n * scale), color="tab:green")

    ax3.set_title("3D Path")
    ax3.set_xlabel("X")
    ax3.set_ylabel("Y")
    ax3.set_zlabel("Z")
    equal_axes_3d(ax3, pts)

    # local bed-plane projection
    rel = pts - origin
    uu = rel @ u
    vv = rel @ v
    ax2.plot(uu, vv, linewidth=1.4)
    ax2.scatter([uu[0]], [vv[0]], c="green", s=40)
    ax2.scatter([uu[-1]], [vv[-1]], c="red", s=40)
    ax2.set_aspect("equal", "box")
    ax2.grid(True, alpha=0.3)
    ax2.set_xlabel(f"u ({axes['u_name']}) [m]")
    ax2.set_ylabel(f"v ({axes['v_name']}) [m]")
    ax2.set_title("Bed Plane Projection")

    fig.suptitle(
        f"pattern={pattern}, {subtitle}, normal={axes['normal_axis']} sign={axes['normal_sign']}\n"
        f"start={np.round(pts[0],4)}, end={np.round(pts[-1],4)}"
    )
    fig.tight_layout()

    if args.save:
        save_path = args.save
        if os.path.isdir(save_path):
            save_path = os.path.join(save_path, "planar_path_debug.png")
        elif save_path.endswith("/"):
            os.makedirs(save_path, exist_ok=True)
            save_path = os.path.join(save_path, "planar_path_debug.png")
        fig.savefig(save_path, dpi=140)
        print(f"saved: {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
