#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""#48 调参准则可满足性预检 + 结构化声明的唯一读取入口。

背景（#46 / 冻结文档 §2、§4.2）：调参准则只写在 issue 正文与一次性驱动脚本里，
没有任何机制在开跑前检查"准则对参考配置自身是否可满足"。准则①"调参集两场都
finished"把参考配置自己判死，27 组候选被同一条件无差别丢弃——诚实执行者必然
"无可采纳"，不诚实执行者可以随手放宽判据放行任意赢家。两种坏结果源于同一处缺检。

本脚本把「搜索空间 + 准则 + 参考配置」变成一份结构化声明
（config/mpc_tuning_declaration.json，fsac.tuning.declaration/v1），并提供：

  check      开跑前的机械预检：声明结构自洽；参考配置逐字与 qp_solver.hpp 对账；
             比较集命令不含调参旋钮（#45 第 8 节）；每条准则在参考配置上求值，
             不成立且未显式 `waived: <理由>` → 拒绝开跑并指名该条（退出码 1）。
  commands   从声明枚举调参网格的 runner 命令行（驱动脚本从这里读，不再硬编码）。
  evaluate   对已跑完的候选产物逐条求值准则，输出判定表 + 平手排序（改动最小优先）。
  emit-doc   由声明生成 docs/MPC_TUNING_FREEZE.md 的 AUTO-DECLARATION 区段。
  check-doc  校验冻结文档里的区段与声明一致（偏差即报错）；§1 参考配置行同样对账。
  selftest   纯 stdlib unittest 自测（正/负样本，不跑 benchmark_runner）。

约定：求值只用确定性计数类指标（run_status / rmse / 圈速 / failures / holds），
`solve_time_ms` 百分位跨运行抖动，不得进入任何准则（冻结文档 §3 方法学修正）。
"""
from __future__ import annotations

import argparse
import json
import re
import shlex
import subprocess
import sys
import tempfile
import unittest
from itertools import product
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DECLARATION = "config/mpc_tuning_declaration.json"
FREEZE_DOC = "docs/MPC_TUNING_FREEZE.md"
DECLARATION_SCHEMA = "fsac.tuning.declaration/v1"

MARK_BEGIN_TPL = "<!-- BEGIN AUTO-DECLARATION（由 scripts/tuning_precheck.py emit-doc 从 {decl} 生成，勿手改本区段） -->"
MARK_END = "<!-- END AUTO-DECLARATION -->"

# 指标方向：max = 越大越好，min = 越小越好。准则里只能用这些确定性指标。
METRIC_DIRECTIONS = {
    "run_status_rank": "max",
    "rmse_lateral_m": "min",
    "lap_time_s": "min",
    "failure_rate": "min",
    "accepted_approx_rate": "min",
    "converged_rate": "max",
}
RUN_STATUS_RANK = {
    "finished": 2,
    "incomplete": 1,
    "failed": 0,
    "timed_out": 0,
    "stopped": 0,
}
# #45 第 8 节：比较集命令禁止出现的调参旋钮。
FORBIDDEN_KNOB_FLAGS = {
    "--mpc-eps-abs",
    "--mpc-eps-rel",
    "--mpc-rho",
    "--mpc-max-iter",
    "--corridor-scale",
}
VALID_OPS = {"not_worse", "equals", "at_most", "at_least", "strictly_better_some"}
VALID_KINDS = {"filter", "tiebreak"}


class DeclError(Exception):
    """声明文件不合法 / 与事实源不一致 / 结构缺项——预检的 fail-closed 通道。"""


# ---------------------------------------------------------------- 声明加载与校验

def load_declaration(path: Path) -> dict:
    try:
        decl = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise DeclError(f"声明文件不存在：{path}") from exc
    except json.JSONDecodeError as exc:
        raise DeclError(f"声明文件不是合法 JSON：{path}: {exc}") from exc
    validate_declaration(decl)
    return decl


def _require(cond: bool, msg: str) -> None:
    if not cond:
        raise DeclError(msg)


def validate_declaration(decl: dict) -> None:
    _require(decl.get("schema") == DECLARATION_SCHEMA,
             f"schema 必须为 {DECLARATION_SCHEMA}，实际 {decl.get('schema')!r}")
    for key in ("reference_config", "knob_cli", "tuning_set", "comparison_set",
                "search_space", "criteria"):
        _require(key in decl, f"声明缺少必需字段：{key}")
    ref = decl["reference_config"]
    _require("source" in ref and isinstance(ref["fields"], dict) and ref["fields"],
             "reference_config 必须含 source 与非空 fields")
    _require(isinstance(decl["tuning_set"], list) and decl["tuning_set"],
             "tuning_set 必须是非空列表")
    keys = [sc["key"] for sc in decl["tuning_set"]]
    _require(len(set(keys)) == len(keys), "tuning_set 场景 key 重复")
    for sc in decl["tuning_set"] + decl["comparison_set"]:
        _require(isinstance(sc.get("args"), list), f"场景 {sc.get('key')} 缺 args 列表")
    grid = decl["search_space"].get("grid", {})
    for knob in grid:
        _require(knob in decl["knob_cli"], f"网格维度 {knob} 没有对应的 runner 旋钮（knob_cli）")
    for crit in decl["criteria"]:
        _require(crit.get("id") and crit.get("kind") in VALID_KINDS,
                 f"准则缺 id/kind：{crit}")
        if crit["kind"] != "filter":
            continue
        _require(crit.get("op") in VALID_OPS,
                 f"准则 {crit['id']} 的 op 未知：{crit.get('op')!r}")
        if crit.get("op") == "strictly_better_some":
            ms = crit.get("metrics")
            _require(isinstance(ms, list) and ms and all(x in METRIC_DIRECTIONS for x in ms),
                     f"准则 {crit['id']} 的 metrics 必须是非空确定性指标列表：{ms!r}")
        else:
            _require(crit.get("metric") in METRIC_DIRECTIONS,
                     f"准则 {crit['id']} 的 metric 不在确定性指标表：{crit.get('metric')!r}")
        scope = crit.get("scope", "all_tuning")
        _require(scope == "all_tuning" or scope in keys,
                 f"准则 {crit['id']} 的 scope 指向不存在的调参场景：{scope!r}")
        if crit["op"] not in ("not_worse", "strictly_better_some"):
            _require("value" in crit, f"绝对准则 {crit['id']} 缺 value")
        waived = crit.get("waived")
        _require(waived is None or (isinstance(waived, str) and waived.strip()),
                 f"准则 {crit['id']} 的 waived 必须是省略或带理由的非空字符串")


def check_reference_matches_source(decl: dict, root: Path) -> None:
    """参考配置 = qp_solver.hpp 现值：逐字段对源码默认值文本求值，漂移即拒。"""
    src_rel = decl["reference_config"]["source"]
    src = root / src_rel
    _require(src.is_file(), f"参考配置声明的事实源不存在：{src_rel}")
    text = src.read_text(encoding="utf-8")
    for field, spec in decl["reference_config"]["fields"].items():
        cpp_field = spec.get("cpp_field", field)  # 声明维度名与 C++ 成员名可解耦（如 acceptable_primal → *_residual）
        m = re.search(rf"\b{re.escape(cpp_field)}\s*\{{([^}}]*)\}}", text)
        _require(m is not None, f"{src_rel} 里找不到成员 {cpp_field} 的默认值，声明与代码已脱节")
        try:
            got = float(m.group(1))
            want = float(spec["value"])
        except ValueError as exc:
            raise DeclError(f"{src_rel}:{field} 默认值无法解析：{m.group(1)!r}") from exc
        _require(got == want,
                 f"声明的参考配置与源码不一致：{field} 声明 {spec['value']}，{src_rel} 现值 {m.group(1)}")


def check_comparison_set_clean(decl: dict) -> None:
    for sc in decl["comparison_set"]:
        bad = [a for a in sc["args"] if a in FORBIDDEN_KNOB_FLAGS]
        _require(not bad,
                 f"比较集场景 {sc['key']} 的命令含调参旋钮 {bad}（#45 第 8 节：比较阶段禁止）")


# ---------------------------------------------------------------- 指标提取

def _fnum(d: dict, key: str):
    v = d.get(key)
    return float(v) if isinstance(v, (int, float)) else None


def read_run_metrics(kpi_path: Path, diag_path: Path | None) -> dict:
    kpi = json.loads(Path(kpi_path).read_text(encoding="utf-8"))
    m = {
        "run_status": kpi.get("run_status"),
        "run_status_rank": RUN_STATUS_RANK.get(kpi.get("run_status"), 0),
        "controller_name": kpi.get("controller_name"),
        "rmse_lateral_m": _fnum(kpi, "rmse_lateral_m"),
        "lap_time_s": None,
        "failure_rate": None,
        "accepted_approx_rate": None,
        "converged_rate": None,
    }
    lap = _fnum(kpi, "best_valid_lap_time_s") or 0.0
    cur = _fnum(kpi, "current_lap_time_s") or 0.0
    m["lap_time_s"] = lap if lap > 0 else (cur if cur > 0 else None)
    if diag_path is not None and Path(diag_path).is_file():
        diag = json.loads(Path(diag_path).read_text(encoding="utf-8"))
        solves = _fnum(diag, "solves")
        failures = _fnum(diag, "failures")
        if solves and solves > 0 and failures is not None:
            m["failure_rate"] = failures / solves
            conv = _fnum(diag, "converged")
            approx = _fnum(diag, "accepted_approx")
            if conv is not None and approx is not None:
                _require(failures + conv + approx == solves,
                         f"{diag_path}: 诊断分解不满足 failures+converged+accepted_approx == solves，"
                         "计数已损坏，拒绝求值")
                m["converged_rate"] = conv / solves
                m["accepted_approx_rate"] = approx / solves
    return m


def expected_controller(sc: dict):
    """从场景 args 里取 --controller 值；拼参出错（如整串 args 未被分词）时，
    KPI 里的 controller_name 会回落成 pure_pursuit —— 比错误判据更响的失败。"""
    args = sc.get("args", [])
    if "--controller" in args:
        return args[args.index("--controller") + 1]
    return None


def assert_run_matches_scenario(sc: dict, metrics: dict, where: str) -> None:
    exp = expected_controller(sc)
    if exp is not None:
        got = metrics.get("controller_name")
        got_norm = {"PurePursuit": "pure_pursuit", "MPC": "mpc"}.get(got, got)
        _require(got_norm == exp,
                 f"{where}: 产物 controller_name={got!r} 与场景要求的 {exp!r} 不符——"
                 "命令拼装/分词出错，这一跑不是声明口径，拒绝求值")


METRIC_LABEL_ZH = {
    "run_status_rank": "终态（finished>incomplete>failed）",
    "rmse_lateral_m": "rmse (m)",
    "lap_time_s": "用时/圈速 (s)",
    "failure_rate": "未收敛率（拒收拍占比）",
    "accepted_approx_rate": "兜底接受率",
    "converged_rate": "严格收敛率",
}


# ---------------------------------------------------------------- 准则求值

def _metric_pair(crit: dict, ref_m: dict, cand_m: dict):
    scope = crit.get("scope", "all_tuning")
    if crit["op"] == "not_worse":
        return ref_m.get(crit["metric"]), cand_m.get(crit["metric"])
    return crit["value"], cand_m.get(crit["metric"])


def eval_filter_on_reference(crit: dict, ref_metrics: dict) -> tuple:
    """预检：准则在**参考配置自身**上成不成立。not_worse 自反必成立；
    strictly_better_some 对基线自身永不可满足（⇒ 声明必须带显式 waived，这是对
    "改参数必须有收益"语义的诚实表达）；绝对 op 逐场景比对参考指标。"""
    if crit["op"] == "not_worse":
        return True, ["不劣于参考 ⇒ 参考配置平凡满足"]
    if crit["op"] == "strictly_better_some":
        return False, ["严格改进对参考基线自身定义上不可满足 ⇒ 需声明里显式 waived（豁免仅作用于预检）"]
    scopes = (crit.get("scope", "all_tuning"),)
    if scopes[0] == "all_tuning":
        scopes = tuple(ref_metrics.keys())
    notes = []
    ok = True
    for key in scopes:
        got = ref_metrics[key].get(crit["metric"])
        want = crit["value"]
        direction = METRIC_DIRECTIONS[crit["metric"]]
        if got is None:
            ok = False
            notes.append(f"{key}: 指标缺失（无法判定）")
            continue
        if crit["op"] == "equals":
            hit = got == want
        elif crit["op"] == "at_least":
            hit = got >= want
        else:  # at_most
            hit = got <= want
        notes.append(f"{key}: 参考值 {got} vs 要求 {crit['op']} {want} → {'成立' if hit else '不成立'}")
        ok = ok and hit
    return ok, notes


def eval_filter_on_candidate(crit: dict, ref_metrics: dict, cand_metrics: dict) -> tuple:
    """正式判定：对候选产物逐场景求值一条 filter 准则。"""
    scope = crit.get("scope", "all_tuning")
    keys = tuple(ref_metrics.keys()) if scope == "all_tuning" else (scope,)
    if crit["op"] == "strictly_better_some":
        notes, hit_any = [], False
        for key in keys:
            for metric in crit["metrics"]:
                ref_v, cand_v = ref_metrics[key].get(metric), cand_metrics[key].get(metric)
                if ref_v is None or cand_v is None:
                    continue
                direction = METRIC_DIRECTIONS[metric]
                better = cand_v < ref_v if direction == "min" else cand_v > ref_v
                if better:
                    notes.append(f"{key}: {METRIC_LABEL_ZH.get(metric, metric)} {cand_v} 严格优于参考 {ref_v}")
                    hit_any = True
        if not hit_any:
            notes.append("无任何指标严格优于参考 ⇒ 改参数无收益，拒")
        return hit_any, notes
    direction = METRIC_DIRECTIONS[crit["metric"]]
    notes, ok = [], True
    for key in keys:
        ref_v, cand_v = ref_metrics[key].get(crit["metric"]), cand_metrics[key].get(crit["metric"])
        if crit["op"] != "not_worse":
            want = crit["value"]
            if cand_v is None:
                hit = False
                note = f"{key}: 指标缺失，判不成立（要求 {crit['op']} {want}）"
            elif crit["op"] == "equals":
                hit = cand_v == want
                note = f"{key}: {cand_v} vs {crit['op']} {want} → {'OK' if hit else 'FAIL'}"
            elif crit["op"] == "at_least":
                hit = cand_v >= want
                note = f"{key}: {cand_v} >= {want} → {'OK' if hit else 'FAIL'}"
            else:
                hit = cand_v <= want
                note = f"{key}: {cand_v} <= {want} → {'OK' if hit else 'FAIL'}"
        else:
            if ref_v is None and cand_v is None:
                hit, note = True, f"{key}: 双方指标均缺失（n/a，判等）"
            elif ref_v is None:
                hit, note = True, f"{key}: 参考缺失，候选 {cand_v}（n/a 放行，报告留痕）"
            elif cand_v is None:
                hit, note = False, f"{key}: 候选缺失而参考为 {ref_v} → 劣化"
            else:
                hit = cand_v <= ref_v if direction == "min" else cand_v >= ref_v
                note = f"{key}: 候选 {cand_v} vs 参考 {ref_v}（{'越低越好' if direction == 'min' else '越高越好'}）→ {'OK' if hit else 'FAIL'}"
        notes.append(note)
        ok = ok and hit
    return ok, notes


def change_magnitude(knobs: dict, ref_fields: dict) -> tuple:
    """平手排序键：改动旋钮数 → 相对改动量之和 → 组合号（确定性）。"""
    diffs = []
    rel = 0.0
    for name, value in sorted(knobs.items()):
        base = float(ref_fields[name]["value"])
        v = float(value)
        if v != base:
            diffs.append(name)
            rel += abs(v - base) / (abs(base) if base else 1.0)
    return (len(diffs), round(rel, 12), )


# ---------------------------------------------------------------- 网格枚举

def grid_combos(decl: dict) -> list:
    grid = decl["search_space"]["grid"]
    names = list(grid.keys())
    combos = []
    for idx, values in enumerate(product(*(grid[n] for n in names)), start=1):
        knobs = dict(zip(names, values))
        label = ",".join(f"{n}={v}" for n, v in knobs.items())
        combos.append((f"c{idx:02d}", knobs, label))
    return combos


def combo_args(decl: dict, knobs: dict) -> list:
    args = []
    for name, value in knobs.items():
        args += [decl["knob_cli"][name], str(value)]
    return args


# ---------------------------------------------------------------- 文档区段生成

def generate_doc_block(decl: dict, decl_rel: str) -> str:
    ref = decl["reference_config"]
    ref_str = ", ".join(f"{k}={v['value']}" for k, v in ref["fields"].items())
    lines = [MARK_BEGIN_TPL.format(decl=decl_rel), ""]
    lines.append(f"- 声明 schema：`{decl['schema']}` ｜ 轮次：{decl.get('round', '?')} "
                 f"｜ 预注册于（UTC）：{decl.get('declared_at_utc', '?')} ｜ 发现者："
                 f"{decl.get('declared_by', '?')}")
    lines.append(f"- 政策：{decl.get('policy', {}).get('note', '')}")
    lines.append(f"- 参考配置（= `{Path(ref['source']).name}` 现值，预检逐字对账）：`{ref_str}`")
    lines.append(f"- 调参集（{len(decl['tuning_set'])} 场，先于开跑声明）：")
    for sc in decl["tuning_set"]:
        lines.append(f"  - `{sc['key']}`：`benchmark_runner {' '.join(sc['args'])}`")
    lines.append(f"- 保留比较集（冻结后一次性评估，命令**不得**含调参旋钮，#45 §8）：")
    for sc in decl["comparison_set"]:
        lines.append(f"  - `{sc['key']}`：`benchmark_runner {' '.join(sc['args'])}`")
    grid = decl["search_space"]["grid"]
    fixed = decl["search_space"].get("fixed", {})
    total = 1
    for vals in grid.values():
        total *= len(vals)
    dims = " × ".join(f"`{k} ∈ {{{', '.join(vals)}}}`" for k, vals in grid.items())
    lines.append(f"- 搜索空间（{total} 组）：{dims}"
                 + (f"；固定 {', '.join(f'`{k}={v}`' for k, v in fixed.items())}" if fixed else ""))
    lines.append("- 选择准则（先于第一个候选写定，事后不得放宽；`不劣于参考` ⇒ 参考配置自身逐条平凡满足，"
                 "预检会机械证明这一点）：")
    lines.append("")
    lines.append("  | id | 类别 | 判据 | 范围 | 豁免 |")
    lines.append("  | --- | --- | --- | --- | --- |")
    for crit in decl["criteria"]:
        if crit["kind"] == "filter":
            op_zh = {"not_worse": "不劣于参考", "equals": "==", "at_most": "<=", "at_least": ">=",
                     "strictly_better_some": "至少一项严格优于参考"}[crit["op"]]
            if crit["op"] == "strictly_better_some":
                items = "、".join(f"`{METRIC_LABEL_ZH.get(m, m)}`" for m in crit["metrics"])
                judge = f"{items} 至少一项严格优于参考"
            else:
                judge = f"`{METRIC_LABEL_ZH.get(crit['metric'], crit['metric'])}` {op_zh}"
        else:
            judge = crit["name"]
        waived = crit.get("waived")
        lines.append(f"  | {crit['id']} | {crit['kind']} | {judge} | "
                     f"{crit.get('scope', 'all_tuning')} | {waived if waived else '—'} |")
    for gap in decl.get("known_gaps", []):
        lines.append(f"- 已知缺口：{gap}")
    for crit in decl["criteria"]:
        if crit.get("waived"):
            lines.append(f"- 豁免说明（{crit['id']}）：{crit['waived']}")
    lines.append("")
    lines.append(MARK_END)
    return "\n".join(lines)


def doc_ensure_block(text: str, block: str) -> str:
    """把生成区段写入（或校验）文档文本，返回替换后的文本。"""
    start = text.find("<!-- BEGIN AUTO-DECLARATION")
    end = text.find(MARK_END)
    if start == -1 or end == -1 or end < start:
        raise DeclError("冻结文档缺少 AUTO-DECLARATION 区段（运行 emit-doc --write 生成）")
    end += len(MARK_END)
    return text[:start] + block + text[end:]


def run_doc_check(decl: dict, decl_rel: str, doc_path: Path) -> None:
    text = doc_path.read_text(encoding="utf-8")
    block = generate_doc_block(decl, decl_rel)
    current = text[text.find("<!-- BEGIN AUTO-DECLARATION"):text.find(MARK_END) + len(MARK_END)]
    _require("<!-- BEGIN AUTO-DECLARATION" in text and MARK_END in text,
             f"{doc_path}: 找不到 AUTO-DECLARATION 区段")
    _require(current == block,
             f"{doc_path}: AUTO-DECLARATION 区段与声明不一致（偏差即报错，运行 emit-doc --write 重新生成）")
    # §1 的参考配置行与声明对账（#46 轮历史记录，数值必须仍是同一组事实）
    line = next((ln for ln in text.splitlines() if "参考配置" in ln), None)
    if line:
        pairs = dict(re.findall(r"(eps_abs|eps_rel|max_iter|rho)=([0-9eE.+-]+)", line))
        for name, spec in decl["reference_config"]["fields"].items():
            if name in pairs:
                _require(float(pairs[name]) == float(spec["value"]),
                         f"{doc_path} §1 参考配置行 {name}={pairs[name]} 与声明 {spec['value']} 不一致")


# ---------------------------------------------------------------- 子命令

def _load_ref_metrics_from_dir(d: Path) -> dict:
    metrics_file = d / "metrics.json"
    _require(metrics_file.is_file(), f"{d} 里没有 metrics.json（先跑 check 生成参考产物）")
    return json.loads(metrics_file.read_text(encoding="utf-8"))["per_scenario"]


def cmd_check(argv) -> int:
    ap = argparse.ArgumentParser(prog="tuning_precheck.py check", description=__doc__.splitlines()[0])
    ap.add_argument("--declaration", default=DEFAULT_DECLARATION)
    ap.add_argument("--workdir", default="build/tuning_precheck")
    ap.add_argument("--runner", default="./build/track_benchmark/benchmark_runner",
                    help="benchmark_runner 命令（可含多个词），参考配置 = 代码默认值 ⇒ 不传调参旋钮")
    ap.add_argument("--metrics-from", default=None,
                    help="复用既有参考产物目录（含 metrics.json），不重跑")
    args = ap.parse_args(argv)
    root = REPO_ROOT
    decl = load_declaration(root / args.declaration)
    print(f"[1/4] 声明结构校验：{args.declaration} → OK")
    check_reference_matches_source(decl, root)
    print("[2/4] 参考配置 ↔ qp_solver.hpp 现值对账 → OK")
    check_comparison_set_clean(decl)
    print("[3/4] 比较集命令无调参旋钮（#45 §8） → OK")

    ref_dir = Path(args.workdir) / "reference"
    if args.metrics_from:
        ref_metrics = _load_ref_metrics_from_dir(Path(args.metrics_from))
        ref_dir.mkdir(parents=True, exist_ok=True)
        (ref_dir / "metrics.json").write_text(
            json.dumps({"per_scenario": ref_metrics}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        print(f"[4/4] 复用参考指标：{args.metrics_from}")
    else:
        runner = shlex.split(args.runner)
        _require(runner and (Path(runner[0]).exists() or _which(runner[0])),
                 f"找不到 benchmark_runner：{runner[0]!r}（先构建，或用 --runner 指定）")
        ref_dir.mkdir(parents=True, exist_ok=True)
        ref_metrics = {}
        for sc in decl["tuning_set"]:
            kpi = ref_dir / f"{sc['key']}.kpi.json"
            diag = ref_dir / f"{sc['key']}.diag.json"
            cmd = runner + sc["args"] + ["--out", str(kpi), "--diag-out", str(diag)]
            proc = subprocess.run(cmd, capture_output=True, text=True)
            # 参考运行允许出现 --corridor-scale（调参集场景定义自身），但 QP 旋钮必须全默认
            # （runner 未传旋钮时打印 -1，见 benchmark_runner.cpp 的 TUNING-KNOBS-USED 行）。
            m = re.search(r"TUNING-KNOBS-USED[^\n]*?eps_abs=(\S+) eps_rel=(\S+) rho=(\S+) max_iter=(\S+)",
                          proc.stdout + proc.stderr)
            if m and tuple(m.groups()) != ("-1", "-1", "-1", "-1"):
                raise DeclError(f"参考运行 {sc['key']} 竟带上了 QP 调参旋钮 {m.groups()}"
                                "（参考配置 = 代码默认值，不应触发 SetQpSettings）")
            _require(kpi.is_file(), f"参考运行未产出 {kpi}（rc={proc.returncode}）：{proc.stderr[-400:]}")
            m = read_run_metrics(kpi, diag)
            assert_run_matches_scenario(sc, m, f"参考运行 {sc['key']}")
            ref_metrics[sc["key"]] = m
        (ref_dir / "metrics.json").write_text(
            json.dumps({"per_scenario": ref_metrics}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
        print(f"[4/4] 参考配置已在调参集实测：{ref_dir}")

    failures, waived_hits = [], []
    ok_count = 0
    for crit in decl["criteria"]:
        if crit["kind"] != "filter":
            continue
        satisfied, notes = eval_filter_on_reference(crit, ref_metrics)
        tag = "成立" if satisfied else ("已豁免" if crit.get("waived") else "不成立")
        print(f"  准则 {crit['id']}（{crit.get('name') or crit.get('metric', '')}）: {tag}")
        for n in notes:
            print(f"      - {n}")
        if satisfied:
            ok_count += 1
        elif crit.get("waived"):
            waived_hits.append(crit["id"])
        else:
            failures.append(crit["id"])
    if failures:
        print("\n拒绝开跑：以下准则对参考配置自身不成立，且未显式豁免（#48）：")
        for cid in failures:
            print(f"  - {cid}：修正准则规格，或在声明里写明 \"waived\": \"<理由>\"")
        return 1
    print(f"\n预检通过：{ok_count} 条成立"
          + (f"，{len(waived_hits)} 条显式豁免 {waived_hits}" if waived_hits else "，无豁免")
          + "。参考配置指标已缓存，驱动脚本可继续。")
    return 0


def _which(name: str):
    import shutil
    return shutil.which(name)


def cmd_commands(argv) -> int:
    ap = argparse.ArgumentParser(prog="tuning_precheck.py commands")
    ap.add_argument("--declaration", default=DEFAULT_DECLARATION)
    args = ap.parse_args(argv)
    decl = load_declaration(REPO_ROOT / args.declaration)
    check_comparison_set_clean(decl)
    for cid, knobs, label in grid_combos(decl):
        extra = combo_args(decl, knobs)
        for sc in decl["tuning_set"]:
            print(f"{cid}\t{sc['key']}\t{label}\t" + " ".join(sc["args"] + extra))
    return 0


def cmd_evaluate(argv) -> int:
    ap = argparse.ArgumentParser(prog="tuning_precheck.py evaluate")
    ap.add_argument("--declaration", default=DEFAULT_DECLARATION)
    ap.add_argument("--workdir", default="build/tuning_precheck")
    args = ap.parse_args(argv)
    root = REPO_ROOT
    decl = load_declaration(root / args.declaration)
    ref_metrics = _load_ref_metrics_from_dir(Path(args.workdir) / "reference")
    filters = [c for c in decl["criteria"] if c["kind"] == "filter"]
    passed, rows = [], []
    for cid, knobs, label in grid_combos(decl):
        cdir = Path(args.workdir) / "combos" / cid
        cand = {}
        missing = []
        for sc in decl["tuning_set"]:
            kpi = cdir / f"{sc['key']}.kpi.json"
            diag = cdir / f"{sc['key']}.diag.json"
            if not kpi.is_file():
                missing.append(sc["key"])
                continue
            cand[sc["key"]] = read_run_metrics(kpi, diag if diag.is_file() else None)
        if missing:
            rows.append((cid, label, "无产物", f"缺 {missing}"))
            continue
        for sc in decl["tuning_set"]:
            if sc["key"] in cand:
                assert_run_matches_scenario(sc, cand[sc["key"]], f"{cid}/{sc['key']}")
        verdicts = []
        ok = True
        for crit in filters:
            hit, notes = eval_filter_on_candidate(crit, ref_metrics, cand)
            verdicts.append(f"{crit['id']}:{'OK' if hit else 'FAIL'}")
            if not hit:
                ok = False
                reason = "; ".join(n for n in notes if "FAIL" in n or "缺失" in n)
                rows.append((cid, label, f"拒（{crit['id']}）", reason))
                break
        if ok:
            rows.append((cid, label, "通过", " ".join(verdicts)))
            passed.append((change_magnitude(knobs, decl["reference_config"]["fields"]), cid, label))
    passed.sort()
    out = ["| 组合 | 参数 | 判定 | 依据 |", "| --- | --- | --- | --- |"]
    out += [f"| {r[0]} | {r[1]} | {r[2]} | {r[3]} |" for r in rows]
    report = "\n".join(out) + "\n\n平手排序（改动最小优先）：" + (
        ", ".join(cid for _, cid, _ in passed) if passed else "无通过者") + "\n"
    wdir = Path(args.workdir)
    wdir.mkdir(parents=True, exist_ok=True)
    (wdir / "report.md").write_text(report, encoding="utf-8")
    print(report)
    if not passed:
        print("结论：0 组通过全部准则 → 不冻结任何参数变更（如实记录，不得事后放宽）。")
    else:
        print(f"结论：{len(passed)} 组通过；按平手排序建议采纳 {passed[0][1]}（{passed[0][2]}），"
              "仍需在保留比较集上做一次冻结后评估。")
    return 0


def cmd_emit_doc(argv) -> int:
    ap = argparse.ArgumentParser(prog="tuning_precheck.py emit-doc")
    ap.add_argument("--declaration", default=DEFAULT_DECLARATION)
    ap.add_argument("--doc", default=FREEZE_DOC)
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args(argv)
    decl = load_declaration(REPO_ROOT / args.declaration)
    block = generate_doc_block(decl, args.declaration)
    if args.write:
        doc_path = REPO_ROOT / args.doc
        doc_path.write_text(doc_ensure_block(doc_path.read_text(encoding="utf-8"), block),
                            encoding="utf-8")
        print(f"已写入 {args.doc} 的 AUTO-DECLARATION 区段")
    else:
        print(block)
    return 0


def cmd_check_doc(argv) -> int:
    ap = argparse.ArgumentParser(prog="tuning_precheck.py check-doc")
    ap.add_argument("--declaration", default=DEFAULT_DECLARATION)
    ap.add_argument("--doc", default=FREEZE_DOC)
    args = ap.parse_args(argv)
    decl = load_declaration(REPO_ROOT / args.declaration)
    run_doc_check(decl, args.declaration, REPO_ROOT / args.doc)
    print(f"{args.doc}：AUTO-DECLARATION 区段与 {args.declaration} 一致；§1 参考配置行对账通过")
    return 0


def cmd_selftest(argv) -> int:
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(sys.modules[__name__])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


# ---------------------------------------------------------------- 自测（不跑 runner）

def _minimal_decl(**overrides) -> dict:
    decl = {
        "schema": DECLARATION_SCHEMA,
        "round": "t",
        "declared_at_utc": "x",
        "policy": {"note": "test"},
        "reference_config": {
            "source": "fake/qp_solver.hpp",
            "fields": {"eps_abs": {"value": "1e-4"}, "eps_rel": {"value": "1e-4"},
                       "rho": {"value": "1.0"}, "max_iter": {"value": "50"}},
        },
        "knob_cli": {"eps_abs": "--mpc-eps-abs", "eps_rel": "--mpc-eps-rel",
                     "rho": "--mpc-rho", "max_iter": "--mpc-max-iter"},
        "tuning_set": [
            {"key": "acceleration", "args": ["--track", "acceleration"]},
            {"key": "trackdrive_corridor_09", "args": ["--track", "trackdrive", "--corridor-scale", "0.9"]},
        ],
        "comparison_set": [{"key": "trackdrive_v1", "args": ["--track", "trackdrive"]}],
        "search_space": {"grid": {"eps_abs": ["1e-4", "5e-4"], "max_iter": ["50", "150"]},
                         "fixed": {"rho": "1.0"}},
        "criteria": [
            {"id": "C1", "kind": "filter", "name": "终态不劣于参考", "metric": "run_status_rank",
             "op": "not_worse", "scope": "all_tuning"},
            {"id": "C2", "kind": "filter", "name": "rmse 不劣化", "metric": "rmse_lateral_m",
             "op": "not_worse", "scope": "all_tuning"},
        ],
    }
    decl.update(overrides)
    return decl


def _metrics(status="finished", rmse=0.5, lap=10.0, failure_rate=None, approx_rate=None) -> dict:
    return {"run_status": status, "run_status_rank": RUN_STATUS_RANK[status],
            "rmse_lateral_m": rmse, "lap_time_s": lap, "failure_rate": failure_rate,
            "accepted_approx_rate": approx_rate, "converged_rate": None}


class PrecheckTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _write_ref_source(self, text: str) -> None:
        p = self.root / "fake/qp_solver.hpp"
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")

    def test_real_declaration_validates_and_is_clean(self):
        decl = load_declaration(REPO_ROOT / DEFAULT_DECLARATION)
        self.assertEqual(decl["schema"], DECLARATION_SCHEMA)
        check_comparison_set_clean(decl)
        check_reference_matches_source(decl, REPO_ROOT)

    def test_reference_source_drift_rejected(self):
        decl = _minimal_decl()
        self._write_ref_source(
            "double eps_abs{5e-4}; double eps_rel{1e-4}; double rho{1.0}; size_t max_iter{50};")
        with self.assertRaises(DeclError) as ctx:
            check_reference_matches_source(decl, self.root)
        self.assertIn("eps_abs", str(ctx.exception))

    def test_comparison_knob_forbidden_rejected(self):
        decl = _minimal_decl()
        decl["comparison_set"][0]["args"] = ["--track", "trackdrive", "--mpc-eps-abs", "1e-3"]
        with self.assertRaises(DeclError):
            check_comparison_set_clean(decl)

    def test_unsatisfiable_criterion_rejects_and_names_it(self):
        # 验收第 2 条的负样本：绝对准则 run_status>=finished(2)，参考 trackdrive 为 incomplete。
        decl = _minimal_decl()
        decl["criteria"].append({"id": "C9", "kind": "filter", "name": "两场必须 finished（旧①）",
                                 "metric": "run_status_rank", "op": "at_least", "value": 2,
                                 "scope": "all_tuning"})
        ref = {"acceleration": _metrics("finished"), "trackdrive_corridor_09": _metrics("incomplete")}
        ok, _notes = eval_filter_on_reference(decl["criteria"][-1], ref)
        self.assertFalse(ok)

    def test_waiver_allows_unsatisfiable_criterion(self):
        decl = _minimal_decl()
        decl["criteria"].append({"id": "C9", "kind": "filter", "name": "两场必须 finished",
                                 "metric": "run_status_rank", "op": "at_least", "value": 2,
                                 "scope": "all_tuning", "waived": "参考配置本就 incomplete（§2），保留给候选作硬门"})
        ref = {"acceleration": _metrics("finished"), "trackdrive_corridor_09": _metrics("incomplete")}
        ok, _ = eval_filter_on_reference(decl["criteria"][-1], ref)
        self.assertFalse(ok)  # 不成立仍如实话
        self.assertTrue(decl["criteria"][-1].get("waived"))  # 但已豁免 ⇒ cmd_check 放行

    def test_not_worse_self_reflexive_on_reference(self):
        ref = {"acceleration": _metrics("finished", failure_rate=0.1, approx_rate=0.2)}
        for crit in _minimal_decl()["criteria"]:
            ok, _ = eval_filter_on_reference(crit, ref)
            self.assertTrue(ok)

    def test_candidate_worse_rejected_better_or_equal_accepted(self):
        ref = {"acceleration": _metrics("finished", rmse=0.5)}
        crit = _minimal_decl()["criteria"][1]
        worse = {"acceleration": _metrics("finished", rmse=0.7)}
        same = {"acceleration": _metrics("finished", rmse=0.5)}
        ok, _ = eval_filter_on_candidate(crit, ref, worse)
        self.assertFalse(ok)
        ok, _ = eval_filter_on_candidate(crit, ref, same)
        self.assertTrue(ok)

    def test_run_status_regression_rejected(self):
        ref = {"acceleration": _metrics("finished")}
        cand = {"acceleration": _metrics("incomplete")}
        ok, _ = eval_filter_on_candidate(_minimal_decl()["criteria"][0], ref, cand)
        self.assertFalse(ok)

    def test_diag_invariant_enforced(self):
        kpi = self.root / "k.json"
        diag = self.root / "d.json"
        kpi.write_text(json.dumps({"run_status": "finished", "rmse_lateral_m": 0.1,
                                   "best_valid_lap_time_s": 9.0}), encoding="utf-8")
        diag.write_text(json.dumps({"solves": 100, "failures": 10, "converged": 60,
                                    "accepted_approx": 30}), encoding="utf-8")
        m = read_run_metrics(kpi, diag)
        self.assertEqual(m["failure_rate"], 0.1)
        self.assertEqual(m["accepted_approx_rate"], 0.3)
        diag.write_text(json.dumps({"solves": 100, "failures": 10, "converged": 50,
                                    "accepted_approx": 31}), encoding="utf-8")
        with self.assertRaises(DeclError):
            read_run_metrics(kpi, diag)

    def test_doc_block_deterministic_and_drift_detected(self):
        decl = _minimal_decl()
        b1 = generate_doc_block(decl, "d.json")
        b2 = generate_doc_block(decl, "d.json")
        self.assertEqual(b1, b2)
        doc = self.root / "doc.md"
        doc.write_text("# 标题\n\n## 8. 重声明\n\n" + b1 + "\n", encoding="utf-8")
        run_doc_check(decl, "d.json", doc)  # 一致 → 不抛
        decl2 = _minimal_decl()
        decl2["round"] = "changed"
        with self.assertRaises(DeclError):
            run_doc_check(decl2, "d.json", doc)

    def test_reference_line_drift_in_doc_detected(self):
        decl = _minimal_decl()
        block = generate_doc_block(decl, "d.json")
        doc = self.root / "doc.md"
        doc.write_text("## 1. 事先声明\n\n- **参考配置** = `qp_solver.hpp` 现值 "
                       "`(eps_abs=1e-4, eps_rel=1e-4, max_iter=50, rho=1.0)`\n\n" + block + "\n",
                       encoding="utf-8")
        run_doc_check(decl, "d.json", doc)
        decl["reference_config"]["fields"]["eps_abs"]["value"] = "1e-3"
        with self.assertRaises(DeclError):
            run_doc_check(decl, "d.json", doc)

    def test_commands_enumeration_and_tiebreak(self):
        decl = _minimal_decl()
        combos = grid_combos(decl)
        self.assertEqual(len(combos), 4)  # 2×2
        c0 = combo_args(decl, combos[0][1])
        self.assertIn("--mpc-eps-abs", c0)
        ref_fields = decl["reference_config"]["fields"]
        self.assertEqual(change_magnitude({"eps_abs": "1e-4", "max_iter": "50"}, ref_fields)[0], 0)
        self.assertEqual(change_magnitude({"eps_abs": "5e-4", "max_iter": "50"}, ref_fields)[0], 1)

    def test_missing_metric_semantics(self):
        crit = {"id": "CX", "kind": "filter", "metric": "failure_rate", "op": "not_worse",
                "scope": "all_tuning"}
        ref = {"a": _metrics(failure_rate=None)}
        ok, _ = eval_filter_on_candidate(crit, ref, {"a": _metrics(failure_rate=0.1)})
        self.assertTrue(ok)   # 参考缺失 ⇒ n/a 放行但留痕
        ref2 = {"a": _metrics(failure_rate=0.1)}
        ok, _ = eval_filter_on_candidate(crit, ref2, {"a": _metrics(failure_rate=None)})
        self.assertFalse(ok)  # 候选缺失而参考有值 ⇒ 劣化

    def test_controller_guard_rejects_wrong_artifact(self):
        sc = {"key": "a", "args": ["--track", "acceleration", "--controller", "mpc"]}
        bad = _metrics()
        bad["controller_name"] = "PurePursuit"
        with self.assertRaises(DeclError):
            assert_run_matches_scenario(sc, bad, "x")
        good = _metrics()
        good["controller_name"] = "MPC"
        assert_run_matches_scenario(sc, good, "x")
        none_sc = {"key": "b", "args": ["--track", "acceleration"]}
        assert_run_matches_scenario(none_sc, _metrics(), "y")  # 场景未指定 ⇒ 不检查

    def test_strictly_better_some_semantics(self):
        crit = {"id": "C6", "kind": "filter",
                "metrics": ["rmse_lateral_m", "failure_rate"], "op": "strictly_better_some",
                "scope": "all_tuning"}
        ref = {"a": _metrics(rmse=0.5, failure_rate=0.1)}
        ok, _ = eval_filter_on_reference(crit, ref)
        self.assertFalse(ok)  # 对参考自身永远不成立 ⇒ 声明必须带 waived
        equal = {"a": _metrics(rmse=0.5, failure_rate=0.1)}
        hit, notes = eval_filter_on_candidate(crit, ref, equal)
        self.assertFalse(hit)  # 平手拒（防止无收益漂移被采纳）
        better = {"a": _metrics(rmse=0.4, failure_rate=0.1)}
        hit, _ = eval_filter_on_candidate(crit, ref, better)
        self.assertTrue(hit)   # 任一项严格优即过
        worse = {"a": _metrics(rmse=0.9, failure_rate=0.2)}
        hit, _ = eval_filter_on_candidate(crit, ref, worse)
        self.assertFalse(hit)

    def test_validate_rejects_unknown_metric_and_op(self):
        decl = _minimal_decl()
        decl["criteria"][0]["metric"] = "p95_solve_ms"
        with self.assertRaises(DeclError):
            validate_declaration(decl)
        decl = _minimal_decl()
        decl["criteria"][0]["op"] = "fuzzy"
        with self.assertRaises(DeclError):
            validate_declaration(decl)


COMMANDS = {
    "check": cmd_check,
    "commands": cmd_commands,
    "evaluate": cmd_evaluate,
    "emit-doc": cmd_emit_doc,
    "check-doc": cmd_check_doc,
    "selftest": cmd_selftest,
}


def main(argv) -> int:
    if not argv or argv[0] not in COMMANDS:
        print(f"用法: tuning_precheck.py {{{'|'.join(COMMANDS)}}} [选项]\n"
              f"（--help 查看各子命令）", file=sys.stderr)
        return 2
    return COMMANDS[argv[0]](argv[1:])


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
