#!/usr/bin/env python3
"""从 ROS1 .bag 或 ROS2 bag 读取 PointCloud2，仅导出 XYZ JSONL 研究数据。

按 topic 逐条流式读取；不依赖 roscore、ros1_bridge 或 ROS2 安装。
需要：python -m pip install 'rosbags>=0.11,<1'
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path


def decode_xyz(msg, max_points: int = 2500) -> list[list[float]]:
    """解析 PointCloud2 原始 bytes：支持大小端、row padding、任意 xyz 字段顺序。

    浮点 XYZ 允许 FLOAT32(7) / FLOAT64(8)；其他字段不会被解释。
    使用有固定上限的等间隔抽样，便于离线实验但不适合精确密度估计。
    """
    if max_points < 1:
        raise ValueError("max_points 必须为正")
    width, height = int(msg.width), int(msg.height)
    point_step, row_step = int(msg.point_step), int(msg.row_step)
    if min(width, height) < 0 or point_step < 1 or row_step < point_step * width:
        raise ValueError("无效的 PointCloud2 布局")
    buf = memoryview(msg.data).cast("B")
    if len(buf) < row_step * height:
        raise ValueError("PointCloud2.data 长度不足")
    fields = {f.name: f for f in msg.fields}
    endian = ">" if msg.is_bigendian else "<"
    unpackers = {}
    for name in ("x", "y", "z"):
        if name not in fields:
            raise ValueError(f"PointCloud2 缺少 {name} 字段")
        field = fields[name]
        fmt = "f" if field.datatype == 7 else "d" if field.datatype == 8 else None
        if fmt is None or getattr(field, "count", 1) != 1:
            raise ValueError(f"不支持的 {name} 字段类型")
        if field.offset < 0 or field.offset + struct.calcsize(fmt) > point_step:
            raise ValueError(f"{name} 字段越过 point_step")
        unpackers[name] = (struct.Struct(endian + fmt), int(field.offset))

    total = width * height
    stride = max(1, math.ceil(total / max_points))
    out = []
    for flat in range(0, total, stride):
        row, col = divmod(flat, width)
        base = row * row_step + col * point_step
        xyz = [unpackers[k][0].unpack_from(buf, base + unpackers[k][1])[0]
               for k in ("x", "y", "z")]
        if all(math.isfinite(v) for v in xyz):
            out.append(xyz)
    return out


def export_bag(
    bag: Path, output: Path, topic: str = "/velodyne_points",
    max_frames: int = 100, step_frames: int = 1, max_points: int = 2500,
) -> dict:
    if max_frames < 1 or step_frames < 1 or max_points < 1:
        raise ValueError("帧数、步长及点数上限须大于零")
    if not bag.exists():
        raise FileNotFoundError(bag)
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError("缺少 rosbags：python -m pip install 'rosbags>=0.11,<1'") from exc

    count = 0
    seen = 0
    output.parent.mkdir(parents=True, exist_ok=True)
    # 只在明确找到目标 topic 后写输出，防止误生成一个“成功”的空文件。
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS1_NOETIC)) as reader:
        connections = [
            c for c in reader.connections
            if c.topic == topic and c.msgtype.endswith("/PointCloud2")
        ]
        if not connections:
            topics = sorted({f"{c.topic} ({c.msgtype})" for c in reader.connections})
            raise ValueError(f"未找到 PointCloud2 topic {topic}；bag 内 topic：{topics}")
        with output.open("w", encoding="utf-8") as handle:
            for connection, stamp, raw in reader.messages(connections=connections):
                if seen % step_frames:
                    seen += 1
                    continue
                seen += 1
                msg = reader.deserialize(raw, connection.msgtype)
                xyz = decode_xyz(msg, max_points=max_points)
                record = {
                    "frame_id": count, "timestamp_ns": int(stamp),
                    "source_topic": connection.topic,
                    "sensor_frame": str(msg.header.frame_id),
                    "points": xyz,
                }
                handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                count += 1
                if count >= max_frames:
                    break
    if count == 0:
        output.unlink(missing_ok=True)
        raise ValueError("选中 topic 没有可读取的帧")
    return {"frames": count, "output": str(output), "topic": topic,
            "max_points_per_frame": max_points, "frame_stride": step_frames}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="ROS1 *.bag 文件或 ROS2 bag 目录")
    parser.add_argument("--topic", default="/velodyne_points")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=100)
    parser.add_argument("--step-frames", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=2500)
    args = parser.parse_args()
    print(json.dumps(export_bag(
        args.bag, args.out, args.topic, args.max_frames, args.step_frames, args.max_points
    ), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
