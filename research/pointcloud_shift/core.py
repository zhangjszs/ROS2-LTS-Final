"""无 ROS 依赖的点云基线：合成域偏移、简单几何聚类及特征检索诊断。

这里只用于离线研究；它不是 Point-Cache 的复现，也不能替代现有 PCL/ROS2 感知链。
"""
from __future__ import annotations

import json
import math
import random
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path

Point = tuple[float, float, float]
TRUTH_CENTERS = ((5.0, -1.2), (8.0, 1.4), (11.5, -2.2))


@dataclass(frozen=True)
class Cluster:
    x: float
    y: float
    z: float
    size: int
    height: float
    radius: float

    def feature(self) -> tuple[float, float, float]:
        """平移不变的手工几何描述符，仅用作检索诊断。"""
        return (math.log1p(self.size), self.height, self.radius)

    def as_dict(self) -> dict:
        return {
            "center": [round(self.x, 4), round(self.y, 4), round(self.z, 4)],
            "points": self.size,
            "height": round(self.height, 4),
            "radius": round(self.radius, 4),
        }


def synthetic_scene(seed: int) -> tuple[list[Point], list[tuple[float, float]]]:
    """三个锥桶、地面和一根长柱；生成数据，不冒充实车传感器模型。"""
    rng = random.Random(seed)
    points: list[Point] = []
    for cx, cy in TRUTH_CENTERS:
        for _ in range(30):
            angle = rng.uniform(0, 2 * math.pi)
            z = rng.uniform(0.09, 0.49)
            radius = (0.125 * (0.53 - z) / 0.44) * math.sqrt(rng.random())
            points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle), z))
    for _ in range(160):
        points.append((rng.uniform(0.0, 16.0), rng.uniform(-5.0, 5.0), 0.0))
    for i in range(32):
        z = 0.09 + 1.02 * i / 31
        angle = rng.uniform(0, 2 * math.pi)
        points.append((7.0 + 0.03 * math.cos(angle), 3.4 + 0.03 * math.sin(angle), z))
    return points, list(TRUTH_CENTERS)


def shift_points(points: list[Point], kind: str, seed: int) -> list[Point]:
    """只改观测点，不改真值。种子固定以便做可复现实验。"""
    rng = random.Random(seed)
    if kind not in {"clean", "xyz_noise", "dropout", "far_sparse"}:
        raise ValueError(f"未知扰动: {kind}")
    out = []
    for x, y, z in points:
        if kind == "dropout" and rng.random() < 0.67:
            continue
        if kind == "far_sparse" and x > 7.0 and rng.random() < 0.74:
            continue
        if kind == "xyz_noise":
            x += rng.gauss(0, 0.045)
            y += rng.gauss(0, 0.045)
            z += rng.gauss(0, 0.045)
        out.append((x, y, z))
    return out


def clusters(points: list[Point], eps: float = 0.28, min_points: int = 6) -> list[Cluster]:
    """空间哈希 + 26 邻域的确定性欧氏聚类（示范基线，不是生产 PCL 实现）。"""
    if not 0 < eps <= 2 or min_points < 1:
        raise ValueError("eps 须在 (0,2]，min_points >= 1")
    valid = [
        i for i, (x, y, z) in enumerate(points)
        if all(math.isfinite(v) for v in (x, y, z))
        and 0.055 <= z <= 1.25 and math.hypot(x, y) <= 30
    ]
    grid: dict[tuple[int, int, int], list[int]] = defaultdict(list)
    for i in valid:
        x, y, z = points[i]
        grid[(math.floor(x / eps), math.floor(y / eps), math.floor(z / eps))].append(i)

    unseen = set(valid)
    result: list[Cluster] = []
    eps2 = eps * eps
    for start in valid:
        if start not in unseen:
            continue
        unseen.remove(start)
        queue = deque([start])
        members = []
        while queue:
            i = queue.popleft()
            members.append(i)
            x, y, z = points[i]
            key = (math.floor(x / eps), math.floor(y / eps), math.floor(z / eps))
            for a in (-1, 0, 1):
                for b in (-1, 0, 1):
                    for c in (-1, 0, 1):
                        for j in grid.get((key[0] + a, key[1] + b, key[2] + c), ()):
                            if j not in unseen:
                                continue
                            xx, yy, zz = points[j]
                            if (x - xx) ** 2 + (y - yy) ** 2 + (z - zz) ** 2 <= eps2:
                                unseen.remove(j)
                                queue.append(j)
        if len(members) < min_points:
            continue
        cloud = [points[i] for i in members]
        cx = sum(p[0] for p in cloud) / len(cloud)
        cy = sum(p[1] for p in cloud) / len(cloud)
        cz = sum(p[2] for p in cloud) / len(cloud)
        result.append(Cluster(
            cx, cy, cz, len(cloud),
            max(p[2] for p in cloud) - min(p[2] for p in cloud),
            max(math.hypot(p[0] - cx, p[1] - cy) for p in cloud),
        ))
    return result


def cone_candidates(found: list[Cluster]) -> list[Cluster]:
    """仅供实验的几何阈值；对真实障碍物不具备可靠的语义识别能力。"""
    return [c for c in found if 0.13 <= c.height <= 0.67 and c.radius <= 0.24]


def evaluate(predicted: list[Cluster], truth: list[tuple[float, float]], match_radius: float = 0.55) -> dict:
    """同一帧内一对一匹配，避免将多个预测重复算作 TP。"""
    pairs = sorted(
        (math.hypot(c.x - x, c.y - y), i, j)
        for i, c in enumerate(predicted)
        for j, (x, y) in enumerate(truth)
    )
    used_p, used_t = set(), set()
    errors = []
    for error, i, j in pairs:
        if error > match_radius:
            break
        if i not in used_p and j not in used_t:
            used_p.add(i)
            used_t.add(j)
            errors.append(error)
    tp = len(errors)
    fp = len(predicted) - tp
    fn = len(truth) - tp
    return {"tp": tp, "fp": fp, "fn": fn, "center_error_sum_m": sum(errors)}


class FeatureMemory:
    """简单冻结参考缓存：仅报告近邻距离，不参与预测，也不做模型更新。"""

    def __init__(self, capacity: int = 64):
        self.capacity = capacity
        self.items: list[tuple[float, float, float]] = []

    def add(self, found: list[Cluster]) -> None:
        for cluster in found:
            self.items.append(cluster.feature())
            if len(self.items) > self.capacity:
                self.items.pop(0)

    def nearest_distance(self, candidate: Cluster) -> float | None:
        if not self.items:
            return None
        target = candidate.feature()
        return min(math.sqrt(
            (target[0] - ref[0]) ** 2 * 0.25
            + (target[1] - ref[1]) ** 2 * 4.0
            + (target[2] - ref[2]) ** 2 * 9.0
        ) for ref in self.items)


def _report_counts(tp: int, fp: int, fn: int) -> dict:
    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "precision": round(precision, 5),
            "recall": round(recall, 5), "f1": round(f1, 5)}


def run_synthetic(seed: int = 42, frames: int = 12) -> dict:
    if frames < 1:
        raise ValueError("frames 必须大于零")
    # 独立支持集；不使用评测帧的真值来构建缓存。
    memory = FeatureMemory()
    for i in range(3):
        clean, _ = synthetic_scene(seed + 10000 + i)
        memory.add(cone_candidates(clusters(clean)))

    results = {}
    for scenario in ("clean", "xyz_noise", "dropout", "far_sparse"):
        tp = fp = fn = 0
        center_error = 0.0
        retrieval = []
        for i in range(frames):
            raw, truth = synthetic_scene(seed + i)
            observed = shift_points(raw, scenario, seed + 1000 + i)
            pred = cone_candidates(clusters(observed))
            metrics = evaluate(pred, truth)
            tp += metrics["tp"]
            fp += metrics["fp"]
            fn += metrics["fn"]
            center_error += metrics["center_error_sum_m"]
            retrieval.extend(d for c in pred if (d := memory.nearest_distance(c)) is not None)
        results[scenario] = {
            **_report_counts(tp, fp, fn),
            "mean_tp_centroid_error_m": round(center_error / tp, 5) if tp else None,
            "mean_nearest_reference_distance": round(sum(retrieval) / len(retrieval), 5)
            if retrieval else None,
        }
    return {
        "mode": "synthetic", "seed": seed, "frames_per_scenario": frames,
        "ground_truth_available": True,
        "reference_frames": 3, "results": results,
        "limitations": "合成锥桶与手工特征；缓存只检索、不改变预测；非 Point-Cache 复现或实车性能证据",
    }


def inspect_jsonl(path: Path, support_frames: int = 3) -> dict:
    """真实 bag 导出 JSONL 没有标注；严格不输出 precision/recall/F1。"""
    memory = FeatureMemory()
    rows = []
    with path.open(encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, 1):
            if not line.strip():
                continue
            item = json.loads(line)
            if "points" not in item:
                raise ValueError(f"第 {lineno} 行缺少 points")
            xyz = []
            for p in item["points"]:
                if len(p) != 3:
                    raise ValueError(f"第 {lineno} 行点坐标必须为 XYZ")
                xyz.append(tuple(float(v) for v in p))
            found = clusters(xyz)
            candidates = cone_candidates(found)
            distances = [
                d for c in candidates if (d := memory.nearest_distance(c)) is not None
            ]
            rows.append({
                "frame_id": item.get("frame_id", len(rows)),
                "timestamp_ns": item.get("timestamp_ns"),
                "input_points": len(xyz),
                "clusters": len(found),
                "cone_candidates": len(candidates),
                "mean_nearest_reference_distance": round(sum(distances) / len(distances), 5)
                if distances else None,
            })
            if len(rows) <= support_frames:
                memory.add(candidates)
    if not rows:
        raise ValueError("数据文件为空")
    return {
        "mode": "unlabeled-bag", "ground_truth_available": False,
        "frames": len(rows), "support_frames": min(support_frames, len(rows)),
        "per_frame": rows,
        "limitations": "仅几何聚类与特征检索诊断；无真值不能计算检测准确率；运动坐标系变化会混入漂移",
    }
