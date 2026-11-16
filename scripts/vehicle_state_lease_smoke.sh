#!/usr/bin/env bash
# #107：仅软件的双租约/恢复/最终出口故障注入；隔离 DDS 域，不接硬件。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-build/vehicle_state_lease}"
mkdir -p "$OUT"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${LEASE_TEST_DOMAIN_ID:-$((100 + $$ % 100))}"
export ROS_HOME="${ROS_HOME:-$PWD/build/.ros}"
export TMPDIR="${TMPDIR:-$PWD/build/tmp}"
mkdir -p "$ROS_HOME" "$TMPDIR"
# timeout 的退出码也必须进入 JSON，不把超时/依赖缺失伪装成 PASS。
set +e
timeout --signal=TERM --kill-after=5 150 python3 "$SCRIPT_DIR/vehicle_state_lease_smoke.py" "$OUT"
rc=$?
set -e
if [ "$rc" -eq 124 ] || [ "$rc" -eq 137 ]; then
    python3 - "$OUT" "$rc" <<'PY'
import json
import pathlib
import sys
pathlib.Path(sys.argv[1], "summary.json").write_text(json.dumps({
    "status": "FAIL", "reason": "integration wall budget exceeded", "rc": int(sys.argv[2])}, indent=2))
PY
fi
exit "$rc"
