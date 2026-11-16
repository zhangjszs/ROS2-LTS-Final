#!/usr/bin/env bash
# #107：在已有 Jazzy 工作区中用同一 Topic 注入对原始监控 RED、修复监控 GREEN。
# 仅用于隔离 CI/软件测试；只临时替换监控源文件，EXIT 时恢复，绝不改变 Git 分支。
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
OUT="${1:-build/vehicle_state_lease_red_green}"
mkdir -p "$OUT"
BASELINE=8d9479ef930e75192a3391259b1c8486d03a3f79
SOURCE=src/safety/safety_monitor/src/safety_monitor.cpp
BACKUP="$(mktemp "$ROOT/build/issue107-source.XXXXXX")"
cp "$SOURCE" "$BACKUP"
restore_source() { cp "$BACKUP" "$SOURCE"; rm -f "$BACKUP"; }
trap restore_source EXIT
# actions/checkout 默认浅克隆，不假设基线对象已在工作区。
if ! git cat-file -e "$BASELINE:$SOURCE" 2>/dev/null; then
    git fetch --no-tags --depth=1 origin "$BASELINE"
fi
git show "$BASELINE:$SOURCE" >"$SOURCE"
rebuild_monitor() {
    colcon build --symlink-install --packages-select safety_monitor \
        --packages-ignore hello_world huat_launch --cmake-args \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
        -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
}
rebuild_monitor >"$OUT/red-build.log" 2>&1
if bash scripts/vehicle_state_lease_smoke.sh "$OUT/red"; then
    echo 'FAIL: 原始监控未被回归用例证伪'
    exit 1
fi
# 依赖缺失、未发现节点、编译错误等均不能冒充 RED。
python3 - "$OUT/red/summary.json" <<'PY'
import json
import sys
r = json.load(open(sys.argv[1]))
failed = [c for c in r.get("cases", []) if c["status"] == "FAIL"]
assert r["status"] == "FAIL" and len(failed) == 1, r
assert failed[0]["case"] == "one_vehicle_frame_continuous_path", failed
assert any(c["case"] == "startup_no_vehicle" and c["status"] == "PASS" for c in r["cases"]), r
print("EXPECTED RED: 原始监控在单帧定位+持续规划场景失败")
PY
cp "$BACKUP" "$SOURCE"
rebuild_monitor >"$OUT/green-build.log" 2>&1
bash scripts/vehicle_state_lease_smoke.sh "$OUT/green"
echo 'PASS: 同一 Topic 回归已证明原始监控 RED / 修复监控 GREEN（软件证据）'
