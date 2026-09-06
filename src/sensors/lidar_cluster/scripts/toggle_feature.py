#!/usr/bin/env python3
"""
lidar_cluster 快速功能开关测试工具。
用法：
    rosrun lidar_cluster toggle_feature.py profiling on
    rosrun lidar_cluster toggle_feature.py adaptive_voxel on
    rosrun lidar_cluster toggle_feature.py sor on
    rosrun lidar_cluster toggle_feature.py dynamic_roi on
    rosrun lidar_cluster toggle_feature.py ground_autotune on
    rosrun lidar_cluster toggle_feature.py status
"""

import sys
import os
import re

YAML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "lidar_cluster.yaml"
)

FEATURES = {
    "profiling":        ("enable_profiling", "false"),
    "adaptive_voxel":   ("enable_adaptive_voxel", "false"),
    "sor":              ("enable_sor", "false"),
    "dynamic_roi":      ("enable_dynamic_roi", "false"),
    "ground_autotune":  ("enable_ground_autotune", "false"),
    "adaptive_alpha":   ("enable_adaptive_alpha", "false"),  # in launch, not yaml
    "sliding_window":   ("enable_sliding_window", "false"),   # in launch, not yaml
}


def read_yaml():
    with open(YAML_PATH, 'r') as f:
        return f.read()


def write_yaml(content):
    with open(YAML_PATH, 'w') as f:
        f.write(content)


def set_feature(content, key, value):
    pattern = rf"^({re.escape(key)}\s*:\s*)(\S+)$"
    replacement = rf"\g<1>{value}"
    new_content, count = re.subn(pattern, replacement, content, flags=re.MULTILINE)
    if count == 0:
        print(f"[toggle] Warning: key '{key}' not found in yaml")
    return new_content


def show_status(content):
    print("Current lidar_cluster.yaml feature flags:")
    for name, (key, default) in FEATURES.items():
        m = re.search(rf"^{re.escape(key)}\s*:\s*(\S+)$", content, re.MULTILINE)
        val = m.group(1) if m else default
        print(f"  {name:20s} => {val}")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    cmd = sys.argv[1].lower()

    if cmd == "status":
        show_status(read_yaml())
        return

    if len(sys.argv) < 3:
        print("Usage: toggle_feature.py <feature> <on|off>")
        sys.exit(1)

    feature = sys.argv[1].lower()
    state = sys.argv[2].lower()

    if feature not in FEATURES:
        print(f"Unknown feature: {feature}")
        print(f"Available: {', '.join(FEATURES.keys())}")
        sys.exit(1)

    if state not in ("on", "off"):
        print("State must be 'on' or 'off'")
        sys.exit(1)

    key, _ = FEATURES[feature]
    new_val = "true" if state == "on" else "false"

    content = read_yaml()
    content = set_feature(content, key, new_val)
    write_yaml(content)
    print(f"[toggle] Set {feature} ({key}) = {new_val}")
    print(f"[toggle] Remember to 'colcon build --packages-select lidar_cluster' if the node was already running")


if __name__ == '__main__':
    main()
