#!/usr/bin/env bash
# ==============================================================================
# #50：MPC 持续拒解的运行期出口冒烟（每臂一次调用；容差型集成门禁，同批三条同构）。
#
# 臂 A（reject，注入）：MPC 以 eps=1e-14 + max_iter=1 拉起（判据本身一字未改，只是把
#   "收敛"设成不可能），预期：
#     1) /controller/mpc/health 的 rejecting 在预算内点亮，total_rejects>0；
#     2) 点亮之后的**最终出口指令**落在安全侧：racing_status=4、零油门、满制动
#        （#50 上限行为：不再下发"看起来在动"的衰减指令）。
# 臂 B（normal，负样本）：MPC 默认参数拉起，预期整窗口 rejecting 恒为 False，
#   且 total_accepts>0（默认参数在这条回路**真的在求解**——排除"两条臂观测的是同一件事"）。
#
# 拓扑 = sim(trackdrive CSV) → vehicle_state；内联 pathsource 发确定性直线
#   raw_pathlimits → velocity_profiler → /planning/pathlimits → MPC（直发 /vehicle_command，
#   不拉仲裁器，与闭环冒烟同一取舍）；safety_monitor 提供 /system/stop 锁存语义。
#
# 判据全部走内联 rclpy 采样（不用 ros2 topic echo，teardown 有崩溃 bug）；JSON 落盘留证据。
# 进程清理：只按 PID 递进 INT→TERM→KILL。本机 UDP 组播被 VPN 挡时调用前
# export FASTDDS_BUILTIN_TRANSPORTS=SHM（CI 不需要）。
# 用法： bash scripts/mpc_reject_smoke.sh <OUT_DIR> <arm:reject|normal>
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"
: "${TMPDIR:=$PWD/build/tmp}"
export TMPDIR
mkdir -p "$TMPDIR"
# 断言型查询必须走 ros2cli daemon（#49 契约，lint_shell 机检）；先清残留 daemon。
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true

OUT_DIR="${1:-build/mpc_reject_smoke}"
ARM="${2:-reject}"
if [ "$ARM" != "reject" ] && [ "$ARM" != "normal" ]; then
    echo "FAIL: 臂名只能是 reject|normal（当前 $ARM）"
    exit 2
fi
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
LOG_PREFIX="mpc_reject_${ARM}"
rm -f "$OUT_DIR/${LOG_PREFIX}"_*.log "$OUT_DIR/${LOG_PREFIX}.json"

BUDGET_SEC="${BUDGET_SEC:-30}"
REJECT_THRESHOLD="${REJECT_THRESHOLD:-50}"
TRACKS_DIR="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"
TRACK_CSV="$TRACKS_DIR/trackdrive_loop.csv"
[ -f "$TRACK_CSV" ] || { echo "FAIL: 赛道几何缺失 $TRACK_CSV"; exit 1; }

PIDS=()
NODE_PIDS=()
cleanup() {
    local p c alive
    NODE_PIDS=()
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done
    for p in "${NODE_PIDS[@]}" "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    for _ in $(seq 1 8); do
        alive=0
        for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ "$alive" -eq 0 ] && break
        sleep 0.5
    done
    for p in "${NODE_PIDS[@]}" "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 0.5
    for p in "${NODE_PIDS[@]}" "${PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT

start() {
    local log="$1"
    shift
    "$@" >"$log" 2>&1 & PIDS+=($!)
}

echo "=== [$ARM] 拉起 MPC 回路（预算 ${BUDGET_SEC}s，reject_threshold=${REJECT_THRESHOLD}） ==="
start "$OUT_DIR/${LOG_PREFIX}_sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV"
start "$OUT_DIR/${LOG_PREFIX}_safety.log" ros2 run safety_monitor safety_monitor --ros-args -p use_sim_time:=true
start "$OUT_DIR/${LOG_PREFIX}_profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true
if [ "$ARM" = "reject" ]; then
    # 注入：把收敛条件设成物理上不可能（判据表达式与 acceptable_* 一字未改）。
    start "$OUT_DIR/${LOG_PREFIX}_mpc.log" ros2 run mpc_controller mpc_controller_node --ros-args \
        -p use_sim_time:=true -p topics.path:=/planning/pathlimits \
        -p mpc.qp_eps_abs:=1e-14 -p mpc.qp_eps_rel:=1e-14 -p mpc.qp_max_iter:=1 \
        -p diagnostics.reject_threshold:="$REJECT_THRESHOLD"
else
    start "$OUT_DIR/${LOG_PREFIX}_mpc.log" ros2 run mpc_controller mpc_controller_node --ros-args \
        -p use_sim_time:=true -p topics.path:=/planning/pathlimits \
        -p diagnostics.reject_threshold:="$REJECT_THRESHOLD"
fi

required_nodes=(vehicle_simulator safety_monitor velocity_profiler mpc_controller)
ready=0
nodes=""
for _ in $(seq 1 12); do
    nodes="$(timeout 12 ros2 node list 2>/dev/null)"
    missing=0
    for n in "${required_nodes[@]}"; do
        grep -q "$n" <<<"$nodes" || missing=1
    done
    [ "$missing" -eq 0 ] && { ready=1; break; }
    sleep 2
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: 节点未全部注册；最后一次 ros2 node list："
    printf '%s\n' "$nodes" | sed 's/^/  /'
    tail -n 5 "$OUT_DIR/${LOG_PREFIX}"_*.log
    exit 1
fi
echo "OK   拓扑就绪，开始 [$ARM] 采样断言"

# 出口纯度预检：/vehicle_command 的发布者必须恰好 1 个（本臂 MPC）。
# 前序门禁泄漏的 command_arbiter_node 会向本话题发 safeStop 帧，直接污染
# last_cmd_after_rejecting 判据（Nightly 36933923349 假红；断言查询走 daemon，#49）。
pub_info="$(timeout 12 ros2 topic info /vehicle_command 2>/dev/null || true)"
pub_count="$(sed -n 's/^Publisher count: \([0-9]*\)$/\1/p' <<<"$pub_info" | head -1)"
if [ "${pub_count:-0}" != "1" ]; then
    echo "FAIL: /vehicle_command 发布者数=${pub_count:-未知}（应为 1）；疑似跨门禁泄漏进程污染，当前节点："
    timeout 12 ros2 node list 2>/dev/null | sed 's/^/  /'
    exit 1
fi
echo "OK   出口纯度（/vehicle_command 发布者=1）"

python3 - "$OUT_DIR" "$ARM" "$BUDGET_SEC" "$REJECT_THRESHOLD" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatControllerHealth, HuatPathLimits, HuatVehicleCmd
from geometry_msgs.msg import Point

out_dir, arm, budget, threshold = sys.argv[1], sys.argv[2], float(sys.argv[3]), int(sys.argv[4])


class Watch(Node):
    """发布确定性直线路径 + 采样 health/出口指令（与闭环冒烟同一 pathsource 几何）。"""

    def __init__(self):
        super().__init__("mpc_reject_watch")
        self.health = None          # 最新一帧
        self.health_n = 0
        self.ever_rejecting = False
        self.cmd_n = 0
        self.last_cmd = None
        self.last_cmd_after_rejecting = None

        self.create_subscription(HuatControllerHealth, "/controller/mpc/health", self.on_health, 50)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_cmd, 50)

        pub = self.create_publisher(HuatPathLimits, "/planning/raw_pathlimits", 10)

        def tick():
            m = HuatPathLimits()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.path = [Point(x=0.5 * i, y=0.0, z=0.0) for i in range(60)]
            m.replan = False
            pub.publish(m)

        self.create_timer(0.1, tick)

    def on_health(self, m):
        self.health_n += 1
        if m.rejecting:
            self.ever_rejecting = True
        self.health = m

    def on_cmd(self, m):
        self.cmd_n += 1
        self.last_cmd = m
        # "拒解锁入之后的出口帧"：以最近一帧 health.rejecting 为准（50Hz 同拍语义）
        if self.health is not None and self.health.rejecting:
            self.last_cmd_after_rejecting = m


rclpy.init()
w = Watch()
t0 = time.time()
spin_deadline = t0 + budget
early = False
while time.time() < spin_deadline:
    rclpy.spin_once(w, timeout_sec=0.1)
    if arm == "reject" and w.last_cmd_after_rejecting is not None:
        # 再多收 2s 让状态稳定（确认 safe hold 持续，而不是过一帧就丢）
        if time.time() - t0 > 4 and w.ever_rejecting:
            stabilise_until = time.time() + 2.0
            while time.time() < stabilise_until:
                rclpy.spin_once(w, timeout_sec=0.1)
            early = True
            break

h = w.health
report = {
    "schema": "fsac.mpc_reject_smoke/v1",
    "arm": arm,
    "budget_s": budget,
    "threshold": threshold,
    "health_frames": w.health_n,
    "cmd_frames": w.cmd_n,
    "ever_rejecting": w.ever_rejecting,
    "rejecting": bool(h.rejecting) if h else None,
    "consecutive_rejects": int(h.consecutive_rejects) if h else None,
    "total_rejects": int(h.total_rejects) if h else None,
    "total_accepts": int(h.total_accepts) if h else None,
    "last_cmd_after_rejecting": None if w.last_cmd_after_rejecting is None else {
        "steering": int(w.last_cmd_after_rejecting.steering),
        "brake_force": int(w.last_cmd_after_rejecting.brake_force),
        "pedal_ratio": int(w.last_cmd_after_rejecting.pedal_ratio),
        "racing_status": int(w.last_cmd_after_rejecting.racing_status),
    },
}
with open(f"{out_dir}/mpc_reject_{arm}.json", "w") as f:
    json.dump(report, f, indent=2)

fails = []


def check(cond, msg):
    print(("  ✔ " if cond else "  ✘ ") + msg)
    if not cond:
        fails.append(msg)


print(f"=== [{arm}] 判据（health {w.health_n} 帧 / cmd {w.cmd_n} 帧，提前结束={early}） ===")
check(w.health_n > 0, "health 有发布（/controller/mpc/health）")
if arm == "reject":
    check(h is not None and h.rejecting, "rejecting 点亮并锁存在最终 health 帧")
    check(h is not None and h.total_rejects >= threshold, f"total_rejects ≥ 阈值（{h.total_rejects if h else None}）")
    c = report["last_cmd_after_rejecting"]
    check(c is not None, "拒解锁入后仍能看到出口指令（safe hold 帧）")
    if c:
        check(c["pedal_ratio"] == 0, f"出口零油门（{c['pedal_ratio']}）")
        check(c["brake_force"] >= 80, f"出口满制动保持（{c['brake_force']} ≥ emergency 80）")
        check(c["racing_status"] == 4, f"racing_status=4 急停语义（{c['racing_status']}）")
else:
    check(not w.ever_rejecting, "负样本：默认参数全程 rejecting 未点亮")
    check(h is not None and h.total_accepts > 0, f"默认参数真的在求解（total_accepts={h.total_accepts if h else None}）")
    check(h is not None and h.threshold == threshold, "阈值参数生效")

sys.exit(1 if fails else 0)
PY
rc=$?
if [ $rc -ne 0 ]; then
    echo "FAIL: [$ARM] 判据不满足（报告见 $OUT_DIR/${LOG_PREFIX}.json；节点日志 ${OUT_DIR}/${LOG_PREFIX}_*.log）"
else
    echo "PASS: [$ARM]"
fi
exit $rc
