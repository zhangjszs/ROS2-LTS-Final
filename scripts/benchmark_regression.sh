#!/usr/bin/env bash
# ==============================================================================
# #17-B：离线确定性核心回归（CI 门禁）。
# 用 track_benchmark 的 benchmark_runner 跑小规模代表性场景，校验：
#   1) 完成性：每个场景退出 0（直线到达终点 / 闭合赛道完成 require-laps，且 rmse 在容差内）
#   2) 判定正确：JSON 中 run_status=finished、关键判据字段存在
#   3) 可复现：同配置两次运行 JSON 位级一致
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
  "skidpad|--track skidpad --require-laps 1 --rmse-max 0.8 --timeout 400"
  "trackdrive|--track trackdrive --require-laps 1 --rmse-max 0.8 --timeout 400"
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

# 保留 r2 作为可复现证据，删除临时副本
rm -f "${OUT_DIR}/accel_r2.json"

if [ "${fail}" -ne 0 ]; then
  echo "Benchmark regression: FAILED"
  exit 1
fi
echo "Benchmark regression: all scenarios finished + reproducible ✔"
