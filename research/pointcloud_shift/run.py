#!/usr/bin/env python3
"""离线研究入口：合成点云域偏移基线 / 未标注 rosbag JSONL 诊断。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from core import inspect_jsonl, run_synthetic


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    synthetic = sub.add_parser("synthetic", help="无需 rosbag 的可重复合成实验")
    synthetic.add_argument("--seed", type=int, default=42)
    synthetic.add_argument("--frames", type=int, default=12)
    synthetic.add_argument("--out", type=Path)

    bag = sub.add_parser("bag", help="未标注 PointCloud2 JSONL 质量/检索诊断")
    bag.add_argument("--input", type=Path, required=True)
    bag.add_argument("--support-frames", type=int, default=3)
    bag.add_argument("--out", type=Path)

    args = parser.parse_args()
    if args.command == "synthetic":
        result = run_synthetic(seed=args.seed, frames=args.frames)
    else:
        if args.support_frames < 0:
            parser.error("--support-frames 不能小于零")
        result = inspect_jsonl(args.input, support_frames=args.support_frames)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
