#!/usr/bin/env bash
# ==============================================================================
# #17-B：离线确定性核心回归（CI 门禁）。
# 用 track_benchmark 的 benchmark_runner 跑小规模代表性场景，校验：
#   1) 完成性：每个场景退出 0（直线到达终点 / 闭合赛道完成 require-laps，且 rmse 在容差内）
#   2) 判定正确：JSON 中 run_status=finished、关键判据字段存在
#   3) 可复现：同配置两次运行 JSON 位级一致
#   4) 负样本：人工构造的越界/碰桶/反向/未完赛/超时均被评测**正确识别**
#      （退出码非 0 且对应判据字段命中）——即“评测器本身也被评测”（#17 验收第 2 条）
#   5) 单一赛道来源：生成器导出的 CSV 必须与仓内已提交（仿真器就读它）逐字节一致
#      ——防止“名字相同、几何不同”的赛道两处各自演变（#17 交付物第 1 条）
#   6) 基线对账：与 benchmarks/baseline/ 里的历史结果比口径（赛道版本/终态/有效圈/越界
#      必须一致，rmse 与圈速允许 RMSE_TOL / LAP_TOL 比例容差）——评价口径发生变化时
#      必须显式重新录制基线，不得静默漂到另一个基线（#17 验收最后一条）
#   7) 速度口径对比（#19 A 前置）：同赛道、同控制器、同种子下分别跑“曲率限速”与
#      “固定限速”两种参照。两者都必须完赛，且圈速必须可区分——否则“控制器变快”
#      可能只是速度策略不同的假象。
# 纯计算、无 ROS 调度，快且稳。ROS 闭环集成冒烟（含调度、需容差）另设。
# 前提：已 source ROS + 工作区 install/setup.bash（ros2 / benchmark_runner 可用）。
# ==============================================================================
set -uo pipefail

OUT_DIR="${1:-build/benchmark_regression}"
mkdir -p "${OUT_DIR}" || { echo "cannot create ${OUT_DIR}"; exit 1; }

fail=0

# 场景: 名称:参数
SCENARIOS=(
  "acceleration|--track acceleration"
  "skidpad|--track skidpad --require-laps 1 --rmse-max 0.6 --timeout 400"
  "trackdrive|--track trackdrive --require-laps 1 --rmse-max 0.6 --timeout 400"
)

echo "=== 1) 完成性 + 2) 判定正确 ==="
for spec in "${SCENARIOS[@]}"; do
  name="${spec%%|*}"
  args="${spec#*|}"
  # shellcheck disable=SC2086
  ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/${name}.json" ${args} \
    >"${OUT_DIR}/${name}.out" 2>"${OUT_DIR}/${name}.log"
  rc=$?
  status="$(grep -oE '"run_status": "[^"]+"' "${OUT_DIR}/${name}.json" 2>/dev/null | head -1)"
  laps="$(grep -oE '"valid_laps": [0-9]+' "${OUT_DIR}/${name}.json" 2>/dev/null | head -1)"
  if [ "${rc}" -eq 0 ] && printf '%s' "${status}" | grep -q 'finished'; then
    echo "OK   ${name}: rc=0 ${status} ${laps}"
  else
    echo "FAIL ${name}: rc=${rc} ${status:-<no-json>}"
    tail -5 "${OUT_DIR}/${name}.log" 2>/dev/null | sed 's/^/       /'
    fail=1
  fi
done

echo "=== 3) 可复现性（acceleration 双跑位级一致） ==="
# shellcheck disable=SC2086
ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/accel_r1.json" --track acceleration >/dev/null 2>&1
ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/accel_r2.json" --track acceleration >/dev/null 2>&1
if diff -q "${OUT_DIR}/accel_r1.json" "${OUT_DIR}/accel_r2.json" >/dev/null 2>&1; then
  echo "OK   deterministic: two runs byte-identical"
else
  echo "FAIL non-deterministic output"
  diff "${OUT_DIR}/accel_r1.json" "${OUT_DIR}/accel_r2.json" | sed 's/^/       /'
  fail=1
fi

# 故障算子也必须位级可复现（它们基于仿真时间，不引入墙钟依赖）
# shellcheck disable=SC2086
ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/reverse_r1.json" \
  --track trackdrive --inject reverse --require-laps 1 --timeout 60 >/dev/null 2>&1
# shellcheck disable=SC2086
ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/reverse_r2.json" \
  --track trackdrive --inject reverse --require-laps 1 --timeout 60 >/dev/null 2>&1
if diff -q "${OUT_DIR}/reverse_r1.json" "${OUT_DIR}/reverse_r2.json" >/dev/null 2>&1; then
  echo "OK   deterministic: injected fault scenario byte-identical"
else
  echo "FAIL injected scenario is not reproducible"
  diff "${OUT_DIR}/reverse_r1.json" "${OUT_DIR}/reverse_r2.json" | sed 's/^/       /'
  fail=1
fi

echo "=== 4) 负样本：人工构造故障需被评测识别 ==="
# 格式：名称:期望退出码非0:JSON 判据断言(grep -E, 作用于整段 JSON)
NEGATIVES=(
  "offroute|--track acceleration --inject offroute --timeout 60|\"out_of_bounds_events\": [1-9]"
  "cone|--track acceleration --inject cone --timeout 60|\"collision_events\": [1-9]"
  "reverse|--track trackdrive --inject reverse --require-laps 1 --timeout 60|\"net_arc_progress_m\": -"
  "unfinished|--track acceleration --inject stuck --timeout 60|\"best_valid_lap_time_s\": 0,"
  "timeout|--track trackdrive --require-laps 1 --timeout 3|\"run_status\": \"timeout\""
)
for spec in "${NEGATIVES[@]}"; do
  name="${spec%%|*}"
  rest="${spec#*|}"
  args="${rest%%|*}"
  want="${rest##*|}"
  # shellcheck disable=SC2086
  ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/neg_${name}.json" ${args} \
    >/dev/null 2>"${OUT_DIR}/neg_${name}.log"
  rc=$?
  hit=$(grep -oE "${want}" "${OUT_DIR}/neg_${name}.json" 2>/dev/null | head -1)
  if [ "${rc}" -ne 0 ] && [ -n "${hit}" ]; then
    echo "OK   ${name}: rc=${rc} 判据命中 ${hit}"
  else
    echo "FAIL ${name}: rc=${rc}（应为非 0）判据 ${want} -> ${hit:-未命中}"
    tail -3 "${OUT_DIR}/neg_${name}.log" 2>/dev/null | sed 's/^/       /'
    fail=1
  fi
done

# 保留 r2 作为可复现证据，删除临时副本
rm -f "${OUT_DIR}/accel_r2.json"

echo "=== 5) 单一赛道来源：生成器输出 == 已提交 CSV ==="
EXPORT_DIR="${OUT_DIR}/tracks_export"
mkdir -p "${EXPORT_DIR}"
TRACK_SRC="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"
ros2 run track_benchmark benchmark_runner --export-tracks "${EXPORT_DIR}" >/dev/null 2>"${OUT_DIR}/tracks_export.log"
if [ ! -f "${EXPORT_DIR}/tracks.json" ]; then
    echo "FAIL 未能导出 tracks.json（看 ${OUT_DIR}/tracks_export.log）"
    fail=1
else
    for csv in acceleration_track.csv skidpad_track.csv trackdrive_loop.csv; do
        if [ ! -f "${TRACK_SRC}/${csv}" ]; then
            echo "FAIL ${TRACK_SRC}/${csv} 不存在（赛道几何应提交在此）"
            fail=1
            continue
        fi
        if diff -q "${EXPORT_DIR}/${csv}" "${TRACK_SRC}/${csv}" >/dev/null; then
            echo "OK   ${csv}: 与生成器输出一致"
        else
            echo "FAIL ${csv}: 与生成器输出不一致（赛道几何已在两处漂移）"
            diff "${EXPORT_DIR}/${csv}" "${TRACK_SRC}/${csv}" | head -6 | sed 's/^/       /'
            echo "       重新导出： ros2 run track_benchmark benchmark_runner --export-tracks ${TRACK_SRC}"
            fail=1
        fi
    done
fi

if [ "${fail}" -ne 0 ]; then
  echo "Benchmark regression: FAILED"
  exit 1
fi

echo "=== 6) 基线对账（口径不得静默漂移） ==="
BASE_DIR="${BASE_DIR:-benchmarks/baseline}"
RMSE_TOL="${RMSE_TOL:-0.25}"   # rmse 允许的比例变化（当前实测精度仍有抖位空间）
LAP_TOL="${LAP_TOL:-0.25}"     # 圈速允许的比例变化
if [ ! -d "$BASE_DIR" ]; then
    echo "SKIP 无基线目录 $BASE_DIR（首次建立：cp ${OUT_DIR}/{acceleration,skidpad,trackdrive}.json $BASE_DIR/）"
else
    python3 - "$BASE_DIR" "$OUT_DIR" "$RMSE_TOL" "$LAP_TOL" <<'PY'
import json, os, sys
base_dir, out_dir, rmse_tol, lap_tol = sys.argv[1:5]
rmse_tol, lap_tol = float(rmse_tol), float(lap_tol)
fails = []
for name in ("acceleration", "skidpad", "trackdrive"):
    bp, op = os.path.join(base_dir, name + ".json"), os.path.join(out_dir, name + ".json")
    if not os.path.isfile(bp):
        fails.append(f"{name}: 基线缺失 {bp}")
        continue
    if not os.path.isfile(op):
        fails.append(f"{name}: 本轮无输出 {op}")
        continue
    b, o = json.load(open(bp)), json.load(open(op))
    # 硬口径：赛道版本、终态、有效圈数、越界/碰撞事件数必须一致
    for k in ("track_version", "run_status", "valid_laps", "out_of_bounds_events"):
        if b.get(k) != o.get(k):
            fails.append(f"{name}: {k} 从 {b.get(k)!r} 变到 {o.get(k)!r}")
    # 软口径：rmse / 圈速 在比例容差内
    for k, tol in (("rmse_lateral_m", rmse_tol), ("best_valid_lap_time_s", lap_tol)):
        bv, ov = float(b.get(k) or 0), float(o.get(k) or 0)
        if bv == 0 and ov == 0:
            continue
        ref = bv if bv else 1.0
        if abs(ov - bv) / ref > tol:
            fails.append(f"{name}: {k} {bv} -> {ov} 超出比例容差 {tol}")
        else:
            print(f"OK   {name}: {k} {bv} -> {ov}")
    if not fails:
        print(f"OK   {name}: 口径与基线一致 (track_version={o.get('track_version')})")
if fails:
    print("FAIL 基线对账不通过：")
    for f in fails:
        print("       -", f)
    print("       如果是有意改进口径/提升控制器：先重新录制基线并在 PR 里说明。")
    sys.exit(1)
PY
    [ $? -eq 0 ] || fail=1
fi

echo "=== 7) 速度口径对比（#19 A：同控制器/同赛道/同种子，只改速度参照） ==="
ros2 run track_benchmark benchmark_runner --out "${OUT_DIR}/trackdrive_constspeed.json" \
  --track trackdrive --require-laps 1 --timeout 400 --speed-source constant --const-speed 5 \
  >/dev/null 2>"${OUT_DIR}/trackdrive_constspeed.log"
cst_rc=$?
cst_lap=$(grep -oE '"best_valid_lap_time_s": [0-9.]+' "${OUT_DIR}/trackdrive_constspeed.json" | grep -oE '[0-9.]+')
cur_lap=$(grep -oE '"best_valid_lap_time_s": [0-9.]+' "${OUT_DIR}/trackdrive.json" | grep -oE '[0-9.]+')
if [ "${cst_rc}" -ne 0 ]; then
    echo "FAIL 固定限速口径未完赛（rc=${cst_rc}）"
    tail -3 "${OUT_DIR}/trackdrive_constspeed.log" | sed 's/^/       /'
    fail=1
elif [ -z "${cst_lap:-}" ] || [ -z "${cur_lap:-}" ]; then
    echo "FAIL 两份结果里缺少 best_valid_lap_time_s（固定 ${cst_lap:-<none>}s vs 曲率 ${cur_lap:-<none>}s）"
    fail=1
elif python3 -c "import sys; sys.exit(0 if float('${cst_lap}') > float('${cur_lap}') else 1)"; then
    echo "OK   两种口径均可完赛且可区分：曲率限速 ${cur_lap}s vs 固定限速(5m/s) ${cst_lap}s"
else
    echo "FAIL 两种速度口径圈速无法区分（曲率 ${cur_lap}s vs 固定 ${cst_lap}s）——--speed-source 未生效？"
    fail=1
fi

if [ "${fail}" -ne 0 ]; then
  echo "Benchmark regression: FAILED"
  exit 1
fi
echo "Benchmark regression: 正向完成性 + 可复现性 + 负样本判据 + 单一赛道来源 + 基线对账 + 速度口径 全部通过 ✔"
