#!/usr/bin/env python3
"""toolpath.json → 로봇 궤적 실행 (미니 슬라이서 브릿지).

`robot_mini_slicer`가 내보낸 toolpath.json(층별 mm 폴리라인)을 읽어
베드 좌표로 변환 → 기존 PlanarMotionNode로 궤적을 그린다.
익스트루더는 쓰지 않고 **궤적만** (허공/베드 위 hover).

설계 결정(기본값):
  - 자세: 교시 quaternion 고정 (force_tcp_x_down:false와 동일 논리, IK 안정)
  - 경로 간 이동(travel): Z 2mm 들고 이동 (베드 긁힘 방지)
  - 베드 작음 → 슬라이서에서 --fit 45 로 맞춰서 넣을 것

사용:
  # 오프라인 검증(로봇/ROS 불필요): 좌표 매핑/범위만 확인
  python3 scripts/toolpath_layer_motion.py --toolpath ~/robot_mini_slicer/output/star.json --dry-run

  # 실제 실행 (Robot Launch + MoveIt + 베드 교시 상태에서)
  python3 scripts/toolpath_layer_motion.py --toolpath ~/robot_mini_slicer/output/star.json
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from planar_layer_motion import (  # noqa: E402
    PlanarMotionNode, load_bed_frame, make_pose,
    CONFIG_PATH, JOINT_NAMES,
)


def load_toolpath(path):
    with open(os.path.expanduser(path), "r") as f:
        doc = json.load(f)
    if doc.get("units") != "mm":
        raise ValueError(f"expected units=mm, got {doc.get('units')}")
    return doc


def compute_bed(cfg):
    """bed_config → (bed_center, axis_u, axis_v, bed_normal, quat, safe_z, origin_z, phys)."""
    bed_frame = load_bed_frame(cfg)
    if bed_frame is None:
        raise RuntimeError("bed_config.yaml calibration이 없음 — 4점 교시 먼저.")
    origin_pos = bed_frame["origin"]
    axis_u = bed_frame["axis_u"]
    axis_v = bed_frame["axis_v"]
    bed_normal = bed_frame["normal"]
    pw = bed_frame["physical_width"]
    ph = bed_frame["physical_height"]
    bed_center = origin_pos + 0.5 * pw * axis_u + 0.5 * ph * axis_v
    quat = list(cfg["bed"]["origin"]["orientation"])  # 교시 자세 (force_tcp_x_down:false 전제)
    safe_margin = float(cfg["motion"].get("safe_margin", 0.03))
    safe_z = origin_pos[2] + safe_margin
    return dict(center=bed_center, u=axis_u, v=axis_v, n=bed_normal,
               quat=quat, safe_z=safe_z, origin_z=origin_pos[2],
               origin=origin_pos, phys=(pw, ph))


def part_to_world(x_mm, y_mm, z_mm, bed, base_clearance):
    """부품 로컬 mm (XY중심, 밑면 z=0) → world 좌표. 부품 중심을 베드 중심에 정렬."""
    return (bed["center"]
            + (x_mm / 1000.0) * bed["u"]
            + (y_mm / 1000.0) * bed["v"]
            + (base_clearance + z_mm / 1000.0) * bed["n"])


def layer_paths_world(layer, bed, base_clearance):
    """한 층의 각 path를 world Pose 리스트로. [(kind, [Pose,...]), ...]"""
    out = []
    z_mm = layer["z"]
    for p in layer["paths"]:
        poses = [make_pose(part_to_world(x, y, z_mm, bed, base_clearance), bed["quat"])
                 for x, y in p["points"]]
        if len(poses) >= 2:
            out.append((p["kind"], poses))
    return out


def path_bbox_world(paths_world):
    pts = np.array([[ps.position.x, ps.position.y, ps.position.z]
                    for _, poses in paths_world for ps in poses])
    return pts.min(axis=0), pts.max(axis=0)


def travel_to(node, target_pose, lift, max_step, velocity_scale):
    """현재 위치 → target: Z를 lift만큼 들고 수평이동 후 하강 (베드 긁힘 방지)."""
    cur = node.get_current_tcp_pose()
    if cur is None:
        return False
    quat = [target_pose.orientation.x, target_pose.orientation.y,
            target_pose.orientation.z, target_pose.orientation.w]
    tx, ty, tz = target_pose.position.x, target_pose.position.y, target_pose.position.z
    top = max(cur.position.z, tz) + lift
    wps = [
        make_pose([cur.position.x, cur.position.y, top], quat),  # 상승
        make_pose([tx, ty, top], quat),                          # 수평 이동
        make_pose([tx, ty, tz], quat),                           # 하강
    ]
    return node.cartesian_move_segmented(wps, max_step, velocity_scale)


def main():
    ap = argparse.ArgumentParser(description="toolpath.json → 로봇 궤적 실행")
    ap.add_argument("--toolpath", required=True, help="슬라이서 출력 toolpath.json")
    ap.add_argument("--base-clearance", type=float, default=0.005,
                    help="첫 층을 베드면 위 몇 m 띄울지 (기본 5mm, 궤적만이라 hover)")
    ap.add_argument("--travel-lift", type=float, default=0.002,
                    help="경로 간 이동 시 들어올릴 높이 m (기본 2mm)")
    ap.add_argument("--max-layers", type=int, default=None, help="테스트용: 앞쪽 N층만")
    ap.add_argument("--perimeter-only", action="store_true", help="인필 건너뛰고 외곽선만")
    ap.add_argument("--velocity-scale", type=float, default=None, help="속도(기본 config값)")
    ap.add_argument("--dry-run", action="store_true", help="ROS 없이 좌표/범위만 출력")
    a = ap.parse_args()

    with open(CONFIG_PATH, "r") as f:
        cfg = yaml.safe_load(f)
    bed = compute_bed(cfg)
    doc = load_toolpath(a.toolpath)

    layers = doc["layers"]
    if a.max_layers is not None:
        layers = layers[:a.max_layers]

    max_step = float(cfg["motion"]["cartesian_max_step"])
    velocity_scale = a.velocity_scale if a.velocity_scale is not None \
        else float(cfg["motion"]["velocity_scale"])

    # --- 모든 층 world 변환 + 요약 ---
    all_layer_paths = []
    for L in layers:
        pw = layer_paths_world(L, bed, a.base_clearance)
        if a.perimeter_only:
            pw = [(k, ps) for k, ps in pw if k == "perimeter"]
        all_layer_paths.append((L, pw))

    n_paths = sum(len(pw) for _, pw in all_layer_paths)
    flat = [(k, ps) for _, pw in all_layer_paths for k, ps in pw]
    lo, hi = path_bbox_world(flat)
    pw_m, ph_m = bed["phys"]

    print("=" * 56)
    print("  Toolpath → 로봇 궤적")
    print("=" * 56)
    print(f"  파일        : {a.toolpath}")
    print(f"  도형        : {doc.get('meta',{}).get('shape','?')}  bbox_mm={doc.get('meta',{}).get('bbox_mm')}")
    print(f"  층/경로     : {len(all_layer_paths)}층  {n_paths}경로  (인필{'제외' if a.perimeter_only else '포함'})")
    print(f"  베드 크기   : {pw_m*100:.1f}cm × {ph_m*100:.1f}cm")
    print(f"  base clear  : {a.base_clearance*1000:.1f}mm,  travel lift: {a.travel_lift*1000:.1f}mm")
    print(f"  자세(quat)  : {[round(q,4) for q in bed['quat']]}  (교시 고정)")
    print(f"  world bbox X: {lo[0]:.4f} ~ {hi[0]:.4f}")
    print(f"        bbox Y: {lo[1]:.4f} ~ {hi[1]:.4f}")
    print(f"        bbox Z: {lo[2]:.4f} ~ {hi[2]:.4f}")
    print(f"  속도/스텝   : vel={velocity_scale}  max_step={max_step}")

    # 베드 범위 이탈 경고
    span_u = np.array(doc.get("meta", {}).get("bbox_mm", [0, 0, 0]))[0] / 1000.0
    span_v = np.array(doc.get("meta", {}).get("bbox_mm", [0, 0, 0]))[1] / 1000.0
    if span_u > pw_m or span_v > ph_m:
        print(f"  [경고] 부품 XY({span_u*100:.1f}×{span_v*100:.1f}cm)가 베드보다 큼 → 슬라이서 --fit 로 축소 필요")
    print("=" * 56)

    if a.dry_run:
        print("[dry-run] 매핑/범위 확인만. 실제 이동 안 함.")
        return

    # --- 실제 실행 ---
    import rclpy
    rclpy.init()
    node = PlanarMotionNode(
        cartesian_min_fraction=float(cfg["motion"].get("cartesian_min_fraction", 0.99)))
    node.configure_bed_floor(bed["center"], bed["n"],
                             min_clearance=float(cfg["motion"].get("min_bed_clearance", 0.0)))

    # --- 실시간 2D 탑뷰 플롯 (베드 + 계획경로 + 실제 TCP) ---
    o, u, v = bed["origin"], bed["u"], bed["v"]
    pw_m2, ph_m2 = bed["phys"]
    bed_corners = np.array([o, o + pw_m2 * u, o + pw_m2 * u + ph_m2 * v,
                            o + ph_m2 * v, o])[:, :2]
    planned_all = [pose for _, poses in flat for pose in poses]  # 전체 계획 경로(초록)
    vis = cfg.get("visualization", {})
    if bool(vis.get("live_plot", True)):
        node.start_live_plot(bed_corners, planned_all,
                             update_rate_hz=float(vis.get("live_plot_rate_hz", 10.0)))
        print("[시각화] 실시간 플롯 창을 띄웠습니다 (별도 창).")
    node.start_actual_recording(rate_hz=20.0)

    # 1) 교시 자세로 이동
    teach_deg = cfg.get("teach", {}).get("joint_positions_deg", [])
    completed = False
    try:
        if len(teach_deg) == 6:
            print("\n[1] 교시 자세로 이동...")
            if not node.direct_joint_move([np.deg2rad(float(x)) for x in teach_deg],
                                          float(cfg["motion"].get("teach_move_duration", 15.0))):
                print("[중단] 교시 자세 이동 실패."); raise SystemExit
        prefer_joint = bool(cfg["motion"].get("prefer_joint_approach", True))

        planned_waypoints = []
        first_entry = True
        for L, pw in all_layer_paths:
            print(f"\n[Layer {L['index']}] z={L['z']:.1f}mm  경로 {len(pw)}개")
            for pi, (kind, poses) in enumerate(pw):
                planned_waypoints.extend(poses)
                # 첫 경로 진입: 큰 안전접근 / 이후: 소폭 travel-lift
                if first_entry:
                    ok = node.safe_approach(poses[0], bed["safe_z"], max_step,
                                            velocity_scale, prefer_joint=prefer_joint)
                    first_entry = False
                else:
                    ok = travel_to(node, poses[0], a.travel_lift, max_step, velocity_scale)
                if not ok:
                    print(f"  [실패] travel→ path {pi} ({kind}). 스킵."); continue
                # 경로 그리기 (전체 한 번에 Cartesian 계획+실행)
                if not node.cartesian_move(poses, max_step, velocity_scale):
                    print(f"  [실패] path {pi} ({kind}) 실행."); continue
                print(f"  ✓ path {pi} ({kind}, {len(poses)}pts)")
        completed = True
    finally:
        # 마무리: 안전 상승 + 저장
        try:
            cur = node.get_current_tcp_pose()
            if cur is not None:
                node.cartesian_move([make_pose(
                    [cur.position.x, cur.position.y, bed["safe_z"]],
                    bed["quat"])], max_step, min(velocity_scale, 0.12))
        except Exception:
            pass
        node.stop_actual_recording()
        node.stop_live_plot()
        js = node.get_current_joint_state()
        if js is not None:
            pos_by_name = dict(zip(js.name, js.position))
            if all(n in pos_by_name for n in JOINT_NAMES):
                node._publish_hold_current([float(pos_by_name[n]) for n in JOINT_NAMES])

        plot = node.save_execution_plot(planned_waypoints if 'planned_waypoints' in dir() else [],
                                        os.path.expanduser("~/planar_layer_executed.png"))
        if plot:
            print(f"\n[시각화] 실행 경로 저장: {plot}")
        a_csv, c_csv, jplot = node.save_joint_tracking_debug(
            os.path.expanduser("~/planar_layer_debug"))
        print(f"[디버그] actual={a_csv}  cmd={c_csv}  plot={jplot}")
        print("[완료] toolpath 궤적 실행." if completed else "[중단] 중간 종료.")
        rclpy.shutdown()


if __name__ == "__main__":
    main()
