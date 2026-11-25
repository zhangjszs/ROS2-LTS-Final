"""Exercise the replay comparison through its JSON CLI without ROS or a bag."""

import hashlib
import json
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "vehicle_state_replay_diff.py"
ORIGIN = {"east_m": 0, "north_m": 0, "heading_rad": 0, "speed_mps": 0}
WEST = {"east_m": -2, "north_m": 0, "heading_rad": math.pi, "speed_mps": 2}


class VehicleStateReplayDiffTest(unittest.TestCase):
    def setUp(self):
        workspace = tempfile.TemporaryDirectory()
        self.addCleanup(workspace.cleanup)
        self.workspace = Path(workspace.name)
        self.case = self.write_json("case.json", {
            "schema": "fsac.vehicle_state_replay_case/v1",
            "scenario": "warmup-then-west",
            "frame": "enu",
            "reference_source": "After warmup, move 2 m due west in 1 s at 2 m/s from the origin.",
            "input": {"warmup_steps": 1, "offsets_ns": [0, 100_000_000, 1_100_000_000]},
            "abs_tolerances": {"east_m": 0.01, "north_m": 0.01, "heading_rad": 0.01, "speed_mps": 0.01},
            "expected": [{"step": 0, "state": None}, {"step": 1, "state": ORIGIN}, {"step": 2, "state": WEST}],
        })

    def write_json(self, name, document):
        path = self.workspace / name
        path.write_text(json.dumps(document), encoding="utf-8")
        return path

    def trace(self, side, states, **fields):
        return self.write_json(f"{side}.json", {
            "schema": "fsac.vehicle_state_replay_trace/v1",
            "ros_version": side,
            "frame": "enu",
            "case_sha256": hashlib.sha256(self.case.read_bytes()).hexdigest(),
            "kind": "synthetic",
            "samples": [{"step": step, "state": state} for step, state in states.items()],
            **fields,
        })

    def run_cli(self, ros1=None, ros2=None):
        command = [sys.executable, str(SCRIPT), str(self.case)]
        for option, path in (("--ros1", ros1), ("--ros2", ros2)):
            if path is not None:
                command.extend([option, str(path)])
        return subprocess.run(command, capture_output=True, text=True, timeout=5)

    def test_two_matching_captures_preserve_wrapped_heading_and_source_hashes(self):
        left = self.trace("ros1", {0: None, 1: ORIGIN, 2: {**WEST, "heading_rad": -math.pi + 0.001}})
        right = self.trace("ros2", {0: None, 1: ORIGIN, 2: {**WEST, "east_m": -2.005}})
        result = self.run_cli(left, right)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["counts"], {"matches_reference": 6, "contract_mismatch": 0, "evidence_missing": 0})
        self.assertEqual(report["case_sha256"], hashlib.sha256(self.case.read_bytes()).hexdigest())
        self.assertEqual(report["runs"]["ros1"]["kind"], "synthetic")
        self.assertEqual(report["runs"]["ros1"]["sha256"], hashlib.sha256(left.read_bytes()).hexdigest())
        self.assertAlmostEqual(report["steps"][2]["ros2_minus_ros1"]["heading_rad"], -0.001)
        self.assertAlmostEqual(report["steps"][2]["ros2_minus_ros1"]["east_m"], -0.005)

    def test_same_wrong_output_fails_both_sides_at_the_affected_step(self):
        east_instead_of_west = {**WEST, "east_m": 2}
        states = {0: None, 1: ORIGIN, 2: east_instead_of_west}
        result = self.run_cli(self.trace("ros1", states), self.trace("ros2", states))
        self.assertEqual(result.returncode, 1, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["counts"]["contract_mismatch"], 2)
        step = report["steps"][2]
        self.assertEqual(step["ros2_minus_ros1"]["east_m"], 0)
        for side in ("ros1", "ros2"):
            self.assertEqual(step[side]["violations"], ["east_m"])

    def test_missing_step_is_distinct_from_observed_silence(self):
        result = self.run_cli(self.trace("ros1", {0: None, 1: ORIGIN, 2: WEST}),
                              self.trace("ros2", {1: ORIGIN, 2: WEST}))
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["steps"][0]["ros1"]["status"], "matches_reference")
        self.assertEqual(report["steps"][0]["ros2"]["status"], "evidence_missing")
        self.assertEqual(report["counts"]["evidence_missing"], 1)

    def test_unexpected_step_blocks_otherwise_matching_captures(self):
        states = {0: None, 1: ORIGIN, 2: WEST}
        result = self.run_cli(self.trace("ros1", states), self.trace("ros2", {**states, 3: WEST}))
        self.assertEqual(result.returncode, 3, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["unexpected_steps"], {"ros1": [], "ros2": [3]})
        self.assertEqual(report["counts"]["matches_reference"], 6)

    def test_no_capture_and_observed_unexpected_silence_remain_distinct(self):
        result = self.run_cli(ros2=self.trace("ros2", {0: None, 1: ORIGIN, 2: None}))
        self.assertEqual(result.returncode, 1, result.stderr)
        report = json.loads(result.stdout)
        self.assertIsNone(report["runs"]["ros1"])
        self.assertEqual(report["counts"]["evidence_missing"], 3)
        self.assertEqual(report["steps"][2]["ros2"]["violations"], ["publication"])

    def test_rejects_wrong_case_frame_side_and_nonfinite_field(self):
        for fields, message in (
            ({"case_sha256": "0" * 64}, "case_sha256"),
            ({"frame": "map"}, "ENU frame"),
            ({"ros_version": "ros2"}, "expected ros_version ros1"),
            ({"samples": [{"step": 0, "state": {**ORIGIN, "east_m": math.nan}}]}, "invalid JSON constant"),
            ({"samples": [{"step": 0, "state": None}] * 2}, "strictly increasing"),
        ):
            with self.subTest(fields=fields):
                result = self.run_cli(ros1=self.trace("ros1", {}, **fields))
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertEqual(result.stdout, "")
                self.assertIn(message, result.stderr)

    def test_adapter_provenance_is_required_and_cannot_be_mixed_with_synthetic_data(self):
        states = {0: None, 1: ORIGIN, 2: WEST}
        provenance = {
            "kind": "adapter",
            "source_revision": "1" * 40,
            "config_sha256": "2" * 64,
            "command": "schema-test export",
            "environment": "schema-test distro and toolchain",
        }
        result = self.run_cli(ros2=self.trace("ros2", states, **{**provenance, "command": ""}))
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("command must be a nonempty string", result.stderr)

        right = self.trace("ros2", states, **provenance)
        result = self.run_cli(self.trace("ros1", states), right)
        self.assertEqual(result.returncode, 2, result.stderr)
        self.assertIn("cannot be paired", result.stderr)

        result = self.run_cli(self.trace("ros1", states, **provenance), right)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(json.loads(result.stdout)["runs"]["ros2"]["config_sha256"], "2" * 64)


if __name__ == "__main__":
    unittest.main()
