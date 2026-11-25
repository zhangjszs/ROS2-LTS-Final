#!/usr/bin/env python3
"""Compare independent ROS vehicle-state captures against a frozen replay case."""

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path


CASE_SCHEMA = "fsac.vehicle_state_replay_case/v1"
TRACE_SCHEMA = "fsac.vehicle_state_replay_trace/v1"
REPORT_SCHEMA = "fsac.vehicle_state_replay_report/v1"
FIELDS = ("east_m", "north_m", "heading_rad", "speed_mps")
SIDES = ("ros1", "ros2")


def unique_object(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f"duplicate JSON key {key!r}")
        obj[key] = value
    return obj


def reject_constant(token):
    raise ValueError(f"invalid JSON constant {token}")


def load_document(path, schema):
    raw = path.read_bytes()
    try:
        document = json.loads(raw, object_pairs_hook=unique_object, parse_constant=reject_constant)
    except ValueError as exc:
        raise ValueError(f"{path}: {exc}") from exc
    if not isinstance(document, dict) or document.get("schema") != schema:
        raise ValueError(f"{path}: expected schema {schema}")
    if document.get("frame") != "enu":
        raise ValueError(f"{path}: expected normalized ENU frame")
    return document, hashlib.sha256(raw).hexdigest()


def required_string(document, field, label):
    value = document.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label}: {field} must be a nonempty string")
    return value


def finite_fields(values, label):
    if not isinstance(values, dict) or set(values) != set(FIELDS):
        raise ValueError(f"{label}: expected fields {', '.join(FIELDS)}")
    result = {}
    for field in FIELDS:
        value = values[field]
        if type(value) not in (int, float):
            raise ValueError(f"{label}: {field} must be a finite number")
        try:
            number = float(value)
        except OverflowError as exc:
            raise ValueError(f"{label}: {field} must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{label}: {field} must be a finite number")
        result[field] = number
    return result


def read_samples(rows, label):
    if not isinstance(rows, list):
        raise ValueError(f"{label}: expected an array of samples")
    samples = {}
    previous_step = -1
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) != {"step", "state"}:
            raise ValueError(f"{label}[{index}]: expected step and state")
        step = row["step"]
        if type(step) is not int or step <= previous_step:
            raise ValueError(f"{label}[{index}]: step must be a strictly increasing nonnegative integer")
        previous_step = step
        state = row["state"]
        if state is not None:
            state = finite_fields(state, f"{label}[{index}].state")
            if abs(state["heading_rad"]) > math.pi or state["speed_mps"] < 0:
                raise ValueError(f"{label}[{index}]: heading must be within [-pi, pi] and speed nonnegative")
        samples[step] = state
    return samples


def read_trace(path, side, case_hash):
    document, trace_hash = load_document(path, TRACE_SCHEMA)
    if document.get("ros_version") != side:
        raise ValueError(f"{path}: expected ros_version {side}")
    if document.get("case_sha256") != case_hash:
        raise ValueError(f"{path}: case_sha256 differs from the supplied case")
    kind = document.get("kind")
    if kind not in ("synthetic", "adapter"):
        raise ValueError(f"{path}: kind must be synthetic or adapter")
    provenance = {"kind": kind, "sha256": trace_hash}
    if kind == "adapter":
        for field, length in (("source_revision", 40), ("config_sha256", 64)):
            value = required_string(document, field, path)
            if re.fullmatch(rf"[a-f0-9]{{{length}}}", value) is None:
                raise ValueError(f"{path}: {field} must be {length} lowercase hexadecimal characters")
            provenance[field] = value
        for field in ("command", "environment"):
            provenance[field] = required_string(document, field, path)
    return read_samples(document.get("samples"), f"{path}: samples"), provenance


def state_deltas(actual, reference):
    if actual is None or reference is None:
        return None
    deltas = {field: actual[field] - reference[field] for field in FIELDS}
    deltas["heading_rad"] = math.remainder(deltas["heading_rad"], math.tau)
    return {field: delta if math.isfinite(delta) else None for field, delta in deltas.items()}


def compare_step(observed, step, reference, tolerances):
    if step not in observed:
        return {"status": "evidence_missing"}
    state = observed[step]
    deltas = state_deltas(state, reference)
    if deltas is None:
        failures = [] if state is reference else ["publication"]
    else:
        failures = [field for field, delta in deltas.items() if delta is None or abs(delta) > tolerances[field]]
    return {
        "status": "contract_mismatch" if failures else "matches_reference",
        "state": state,
        "delta_from_reference": deltas,
        "violations": failures,
    }


def compare(case_path, trace_paths):
    case, case_hash = load_document(case_path, CASE_SCHEMA)
    scenario = required_string(case, "scenario", case_path)
    reference_source = required_string(case, "reference_source", case_path)
    if not isinstance(case.get("input"), dict) or not case["input"]:
        raise ValueError(f"{case_path}: input must describe a frozen scenario")
    tolerances = finite_fields(case.get("abs_tolerances"), f"{case_path}: abs_tolerances")
    if min(tolerances.values()) < 0 or tolerances["heading_rad"] > math.pi:
        raise ValueError(f"{case_path}: invalid absolute tolerance")
    expected = read_samples(case.get("expected"), f"{case_path}: expected")
    if not expected:
        raise ValueError(f"{case_path}: expected must contain at least one step")

    samples = {}
    runs = {}
    for side, path in zip(SIDES, trace_paths):
        samples[side], runs[side] = read_trace(path, side, case_hash) if path else ({}, None)
    kinds = {run["kind"] for run in runs.values() if run is not None}
    if len(kinds) > 1:
        raise ValueError("synthetic and adapter captures cannot be paired")

    unexpected = {side: sorted(samples[side].keys() - expected.keys()) for side in SIDES}
    counts = dict.fromkeys(("matches_reference", "contract_mismatch", "evidence_missing"), 0)
    results = []
    for step, reference in expected.items():
        comparisons = {side: compare_step(samples[side], step, reference, tolerances) for side in SIDES}
        for comparison in comparisons.values():
            counts[comparison["status"]] += 1
        results.append({
            "step": step,
            "expected": reference,
            **comparisons,
            "ros2_minus_ros1": state_deltas(comparisons["ros2"].get("state"), comparisons["ros1"].get("state")),
        })

    status = "matches_reference"
    if counts["evidence_missing"] or any(unexpected.values()):
        status = "insufficient_evidence"
    if counts["contract_mismatch"]:
        status = "contract_mismatch"
    return {
        "schema": REPORT_SCHEMA,
        "status": status,
        "scenario": scenario,
        "case_sha256": case_hash,
        "frame": "enu",
        "reference_source": reference_source,
        "abs_tolerances": tolerances,
        "runs": runs,
        "counts": counts,
        "unexpected_steps": unexpected,
        "steps": results,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", type=Path, help="frozen input definition and independent expected outputs")
    parser.add_argument("--ros1", type=Path, help="normalized ROS 1 capture")
    parser.add_argument("--ros2", type=Path, help="normalized ROS 2 capture")
    args = parser.parse_args()
    try:
        report = compare(args.case, (args.ros1, args.ros2))
    except (OSError, ValueError) as exc:
        print(f"vehicle_state_replay_diff: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return {"matches_reference": 0, "contract_mismatch": 1, "insufficient_evidence": 3}[report["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
