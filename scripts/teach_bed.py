#!/usr/bin/env python3
"""
Bed origin teaching script.
1) Move robot TCP to bed corner (origin point) using MoveIt/RViz
2) Run this script: python3 teach_bed.py
3) Current TCP position is saved to config/bed_config.yaml

The current TCP orientation is also saved so replay uses the same reachable
tool pose that was taught.
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
import tf2_ros


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "../config/bed_config.yaml")


def main():
    rclpy.init()
    node = Node("teach_bed")

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    # spin을 돌려야 TF 메시지를 수신함
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    print("[teach_bed] Waiting for TF (5s)...")
    import time
    deadline = time.time() + 5.0
    while time.time() < deadline:
        executor.spin_once(timeout_sec=0.1)

    try:
        transform = tf_buffer.lookup_transform(
            "base_link", "tcp", rclpy.time.Time()
        )
    except Exception as e:
        print(f"[ERROR] Could not get TCP transform: {e}")
        rclpy.shutdown()
        return

    t = transform.transform.translation
    r = transform.transform.rotation

    pos = [round(t.x, 5), round(t.y, 5), round(t.z, 5)]
    quat = [round(r.x, 5), round(r.y, 5), round(r.z, 5), round(r.w, 5)]

    print(f"[teach_bed] TCP position   : {pos}")
    print(f"[teach_bed] TCP orientation: {quat}")

    with open(CONFIG_PATH, "r") as f:
        config = yaml.safe_load(f)

    config["bed"]["origin"]["position"] = pos
    config["bed"]["origin"]["orientation"] = quat

    with open(CONFIG_PATH, "w") as f:
        yaml.dump(config, f, default_flow_style=False, allow_unicode=True)

    print(f"[teach_bed] Saved to {CONFIG_PATH}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
