"""不依赖 ROS / PCL / 第三方包的单测；需 Python 3.10+。"""
from __future__ import annotations

import json
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from core import (  # noqa: E402
    Cluster, FeatureMemory, clusters, cone_candidates, evaluate, inspect_jsonl,
    run_synthetic, shift_points, synthetic_scene,
)
from rosbag_to_frames import decode_xyz, export_bag  # noqa: E402


def field(name, offset, datatype=7):
    return SimpleNamespace(name=name, offset=offset, datatype=datatype, count=1)


def message(points, *, big_endian=False, row_padding=0, fields=None):
    """构造两行的 PointCloud2-like 对象；不能代替真实 rosbag 集成测试。"""
    prefix = ">" if big_endian else "<"
    fields = fields or [field("x", 0), field("y", 4), field("z", 8)]
    width, height, step = 1, len(points), 16
    row_step = step + row_padding
    out = bytearray(row_step * height)
    for row, xyz in enumerate(points):
        for name, value in zip(("x", "y", "z"), xyz):
            pos = next(f.offset for f in fields if f.name == name)
            struct.pack_into(prefix + "f", out, row * row_step + pos, value)
    return SimpleNamespace(width=width, height=height, point_step=step, row_step=row_step,
                           is_bigendian=big_endian, fields=fields, data=bytes(out))


class PointCloudDecoderTests(unittest.TestCase):
    def test_little_endian_and_row_padding(self):
        cloud = message([(1.5, -2, 0.25), (3, 4, 0.5)], row_padding=7)
        self.assertEqual(decode_xyz(cloud), [[1.5, -2, 0.25], [3, 4, 0.5]])

    def test_big_endian_reordered_fields(self):
        fields = [field("z", 0), field("x", 4), field("y", 8)]
        cloud = message([(2.0, 3.0, 4.0)], big_endian=True, fields=fields)
        self.assertEqual(decode_xyz(cloud), [[2, 3, 4]])

    def test_nan_is_dropped(self):
        cloud = message([(1, 2, math.nan), (2, 3, 4)])
        self.assertEqual(decode_xyz(cloud), [[2, 3, 4]])

    def test_sample_cap(self):
        cloud = message([(float(i), 0, 1) for i in range(10)])
        selected = decode_xyz(cloud, max_points=3)
        self.assertEqual(len(selected), 3)
        self.assertEqual([p[0] for p in selected], [0, 4, 8])

    def test_bad_layout_fails_loudly(self):
        cloud = message([(1, 2, 3)])
        cloud.row_step = 4
        with self.assertRaises(ValueError):
            decode_xyz(cloud)
        cloud.row_step = 16
        cloud.data = cloud.data[:5]
        with self.assertRaises(ValueError):
            decode_xyz(cloud)

    def test_missing_fields(self):
        cloud = message([(1, 2, 3)])
        cloud.fields.pop()
        with self.assertRaisesRegex(ValueError, "缺少 z"):
            decode_xyz(cloud)

    def test_nonexistent_bag(self):
        with self.assertRaises(FileNotFoundError):
            export_bag(Path("/does/not/exist.bag"), Path("/tmp/unused.jsonl"))


class ResearchTests(unittest.TestCase):
    def test_synthetic_scene_is_deterministic(self):
        self.assertEqual(synthetic_scene(17), synthetic_scene(17))
        points, _ = synthetic_scene(17)
        self.assertEqual(shift_points(points, "xyz_noise", 11),
                         shift_points(points, "xyz_noise", 11))

    def test_baseline_detects_three_synthetic_cones(self):
        cloud, truth = synthetic_scene(27)
        candidates = cone_candidates(clusters(cloud))
        metrics = evaluate(candidates, truth)
        self.assertEqual((metrics["tp"], metrics["fn"]), (3, 0))

    def test_unique_matching_prevents_double_count(self):
        c = Cluster(0, 0, 0.3, 10, 0.3, 0.1)
        got = evaluate([c, c], [(0, 0)])
        self.assertEqual((got["tp"], got["fp"], got["fn"]), (1, 1, 0))

    def test_reference_memory_empty_and_identity(self):
        c = Cluster(1, 2, 0.2, 9, 0.3, 0.1)
        mem = FeatureMemory()
        self.assertIsNone(mem.nearest_distance(c))
        mem.add([c])
        self.assertEqual(mem.nearest_distance(c), 0.0)

    def test_synthetic_evaluation_counts_and_determinism(self):
        result = run_synthetic(seed=33, frames=4)
        self.assertEqual(result, run_synthetic(seed=33, frames=4))
        self.assertTrue(result["ground_truth_available"])
        for v in result["results"].values():
            self.assertEqual(v["tp"] + v["fn"], 12)
            self.assertGreaterEqual(v["precision"], 0)
            self.assertLessEqual(v["precision"], 1)
        with self.assertRaises(ValueError):
            run_synthetic(frames=0)

    def test_unlabeled_data_cannot_claim_accuracy(self):
        cloud, _ = synthetic_scene(3)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "clouds.jsonl"
            with path.open("w", encoding="utf-8") as file:
                for i in range(2):
                    file.write(json.dumps({"frame_id": i, "points": cloud}) + "\n")
            summary = inspect_jsonl(path)
        self.assertFalse(summary["ground_truth_available"])
        self.assertEqual(summary["frames"], 2)
        self.assertNotIn("precision", json.dumps(summary))
        self.assertIsNotNone(summary["per_frame"][1]["mean_nearest_reference_distance"])


if __name__ == "__main__":
    unittest.main()
