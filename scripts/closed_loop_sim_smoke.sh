#!/usr/bin/env bash
# ==============================================================================
# #17：ROS 闭环集成冒烟（含调度/DDS，判定带容差；与离线位级回归分开）。
#
# 拉起一条真实的多节点闭环（全部 use_sim_time + 仿真器发布 /clock）：
#   pathsource(契约输入 /planning/raw_pathlimits)
#     → velocity_profiler(速度权威 → /planning/pathlimits 带 target_speeds)
#     → pure_pursuit(→ /vehicle_command)
#     → vehicle_simulator(物理积分 → /localization/vehicle_state → 回到 pathsource 视野)
#   safety_monitor → /system/stop（锁存）→ pure_pursuit / vehicle_simulator
#   track_benchmark 订阅 state+command 产出 KPI 报告（md + json）
#
# 覆盖范围（有意划界）：本冒烟验证**接口与控制链路是否真的闭合**——数据在流、车在动、
# 指令帧合法、评测落盘。参考路径由脚本内联发布（确定性几何），因此不依赖
# 感知→锥桶→urinay 那条腿；那条腿属 #18/#19（真实数据与规划质量），本脚本仅**观测并报告**
# 它是否产生锥桶，不作为门禁判据。
#
# 与 #14 的 qos_contract_check.sh 分工：那个脚本只证明“端点与 QoS 存在”；本脚本证明数据真的流动。
#
# 判定采用**容差**而非位级一致（DDS/调度天然抖动）：
#   1) /clock 随仿真时间单调前进 > MIN_CLOCK_ADVANCE_S
#   2) 车速在预算内达到 MIN_SPEED_MPS（回路确实闭合）
#   3) /vehicle_command 有流且帧头/长度/校验和合法（#15 帧契约端到端）
#   4) /planning/pathlimits 携带等长非空 target_speeds（#14 速度载体）
#   5) track_benchmark 报告 JSON 存在且 total_samples > 0（评测接入闭环）
# 任一条不满足即非 0 退出；始终在墙钟预算内自动退出。
#
# 进程清理沿用 a451ae5 的教训：Jazzy 的 `ros2 run` wrapper 不转发信号，因此对 wrapper
# 与节点 PID 各自递进升级 INT→TERM→KILL，且不做无超时裸 wait。
# 数据面断言用内联 rclpy（`ros2 topic echo` 在部分发行版有 teardown 崩溃 bug）。
# 本机注意：与既有门禁脚本一样只设 ROS_LOCALHOST_ONLY；若本机 UDP 组播被 VPN 挡住，
# 需在调用前 export FASTDDS_BUILTIN_TRANSPORTS=SHM（CI 上不需要）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"

OUT_DIR="${1:-build/closed_loop_smoke}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
# 不留下上一轮日志/报告：否则失败时会被旧文件误导，report 存在性断言也会误判。
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json "$OUT_DIR"/*.md

# 可调容差（墙钟预算 / 判据阈值）
BUDGET_SEC="${BUDGET_SEC:-40}"        # 数据面采样的墙钟上限
MIN_CLOCK_ADVANCE_S="${MIN_CLOCK_ADVANCE_S:-5}"
MIN_SPEED_MPS="${MIN_SPEED_MPS:-0.5}"
TRACKS_DIR="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"

PIDS=()       # ros2 run wrapper
NODE_PIDS=()  # wrapper 的直接子进程 = 真实节点

cleanup() {
    local p c alive
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done
    # 先对节点发 INT：让 track_benchmark 在析构里落盘报告（顺序很重要）
    for p in "${NODE_PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    sleep 2
    for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    for _ in $(seq 1 8); do
        alive=0
        for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ "$alive" -eq 0 ] && break
        sleep 0.5
    done
    for p in "${PIDS[@]}" "${NODE_PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 1
    for p in "${PIDS[@]}" "${NODE_PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true   # 已全部 KILL，此处仅回收，不会阻塞
}
trap cleanup EXIT

start() {  # start <logfile> <cmd...>
    local log="$1"
    shift
    "$@" >"$log" 2>&1 & PIDS+=($!)
}

TRACK_CSV="$TRACKS_DIR/trackdrive_loop.csv"
if [ ! -f "$TRACK_CSV" ]; then
    echo "FAIL: 赛道几何缺失 $TRACK_CSV（应由 benchmark_regression.sh 的导出步骤保证）"
    exit 1
fi

echo "=== 拉起闭环（墙钟预算 ${BUDGET_SEC}s，最低车速 ${MIN_SPEED_MPS}m/s） ==="
start "$OUT_DIR/sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV"
start "$OUT_DIR/safety.log" ros2 run safety_monitor safety_monitor --ros-args -p use_sim_time:=true
start "$OUT_DIR/profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true
# PP 直接发到契约出口 /vehicle_command（本冒烟不拉起仲裁器，避免与 sim 回路双发布者）
start "$OUT_DIR/pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
    -p use_sim_time:=true -p topics.vehicle_command:=/vehicle_command
start "$OUT_DIR/benchmark.log" ros2 run track_benchmark track_benchmark_node --ros-args \
    -p use_sim_time:=true -p track_type:=trackdrive -p controller_name:=PurePursuit \
    -p report_file:="$OUT_DIR/closed_loop_report.md"

# 等节点上图（最多 ~12 轮，每轮自带超时）。
# 用 `--no-daemon`：ros2cli daemon 在切换 ROS_HOME/有残留进程时会返回空列表
# （本地实测：节点已上图但 `ros2 node list` 为空），直接查询图则确定。
required_nodes=(vehicle_simulator safety_monitor velocity_profiler pure_pursuit track_benchmark_node)
ready=0
nodes=""
for _ in $(seq 1 12); do
    nodes="$(timeout 12 ros2 node list --no-daemon 2>/dev/null)"
    missing=0
    for n in "${required_nodes[@]}"; do
        grep -q "$n" <<<"$nodes" || missing=1
    done
    [ "$missing" -eq 0 ] && { ready=1; break; }
    sleep 2
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: 闭环节点未全部注册；最后一次 ros2 node list --no-daemon 输出："
    printf '%s\n' "$nodes" | sed 's/^/  /'
    echo "缺失判据节点：${required_nodes[*]}"
    echo "各节点日志尾部："
    tail -n 3 "$OUT_DIR"/*.log
    exit 1
fi
for p in "${PIDS[@]}"; do
    while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
done
echo "OK   ${#required_nodes[@]} 个节点已注册，开始数据面断言"

# 数据面：内联 rclpy 同时充当确定性参考路径发布者 + 观测者，限时采样后明确退出。
python3 - "$OUT_DIR" "$MIN_SPEED_MPS" "$MIN_CLOCK_ADVANCE_S" "$BUDGET_SEC" <<'PY'
import math
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatCarstate, HuatPathLimits, HuatVehicleCmd
from geometry_msgs.msg import Point
from rosgraph_msgs.msg import Clock

out_dir, min_speed, min_clock, budget = (sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))


class ClosedLoop(Node):
    """发布确定性直线路径（契约输入话题），并观测整条回路的实际数据流。"""

    def __init__(self):
        super().__init__("closed_loop_watch")
        self.clock_last = None
        self.clock_advanced = 0.0
        self.speed_max = 0.0
        self.state_n = 0
        self.cmd_n = 0
        self.cmd_bad_frame = 0
        self.cmd_bad_sum = 0
        self.cmd_brake_only = True
        self.raw_path_n = 0
        self.path_n = 0
        self.speeds_n = 0
        self.cone_max = 0

        # 观测
        self.create_subscription(Clock, "/clock", self.on_clock, 50)
        self.create_subscription(HuatCarstate, "/localization/vehicle_state", self.on_state, 50)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_cmd, 50)
        self.create_subscription(HuatPathLimits, "/planning/pathlimits", self.on_path, 50)
        self.create_subscription(HuatPathLimits, "/planning/raw_pathlimits", lambda m: setattr(self, "raw_path_n", self.raw_path_n + 1), 10)
        # 感知腿：只观测、不判据（属 #18/#19 的范围）
        from common_msgs.msg import HuatMap
        self.create_subscription(HuatMap, "/sensors/cones/fused",
                                 lambda m: setattr(self, "cone_max", max(self.cone_max, len(m.cone))), 20)

        # 发布：从仿真器初始位姿 (0,0,0) 沿 +x 的直线（0.5m 间距，60 点 = 30m）。
        # 必须与初始位姿对齐：否则 PP 的 max_crosstrack_m 会判为飞点而拒绝跟迹。
        pub = self.create_publisher(HuatPathLimits, "/planning/raw_pathlimits", 10)

        def tick():
            m = HuatPathLimits()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.path = [Point(x=0.5 * i, y=0.0, z=0.0) for i in range(60)]
            m.replan = False
            pub.publish(m)

        self.create_timer(0.1, tick)

    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if self.clock_last is not None and t > self.clock_last:
            self.clock_advanced += t - self.clock_last
        self.clock_last = t

    def on_state(self, m):
        self.state_n += 1
        self.speed_max = max(self.speed_max, m.v)

    def on_cmd(self, m):
        self.cmd_n += 1
        # #15 帧契约：帧头/长度 + 16 位累加和（帧头与 length 不参与累加和，故两者都要查）
        if (m.head1, m.head2, m.length) != (0xAA, 0x55, 10):
            self.cmd_bad_frame += 1
            return
        s = (m.steering + m.brake_force + m.pedal_ratio + m.gear_position +
             m.working_mode + m.racing_num + m.racing_status) & 0xFFFF
        if s != m.checksum:
            self.cmd_bad_sum += 1
        if m.pedal_ratio > 0:
            self.cmd_brake_only = False

    def on_path(self, m):
        self.path_n += 1
        if m.path and len(m.target_speeds) == len(m.path):
            self.speeds_n += 1


rclpy.init()
w = ClosedLoop()
t0 = time.time()
while time.time() - t0 < budget:
    rclpy.spin_once(w, timeout_sec=0.1)
    if w.clock_advanced > min_clock and w.speed_max >= min_speed and w.cmd_n > 20 and w.speeds_n > 0:
        break

fails = []
print(f"  sim 时间推进       : {w.clock_advanced:.2f}s        (需 > {min_clock}s)")
print(f"  vehicle_state 消息 : {w.state_n}        峰值车速 {w.speed_max:.2f} m/s (需 >= {min_speed})")
print(f"  raw_pathlimits     : {w.raw_path_n}   pathlimits: {w.path_n} (带等长 target_speeds {w.speeds_n})")
print(f"  vehicle_command    : {w.cmd_n}  坏帧 {w.cmd_bad_frame}  坏校验和 {w.cmd_bad_sum}  纯制动 {w.cmd_brake_only}")
print(f"  [观测项] 感知锥桶峰值: {w.cone_max}（感知/规划质量属 #18/#19，本冒烟不判据）")
if w.clock_advanced <= min_clock:
    fails.append("/clock 未按仿真时间推进（publish_clock 或调度异常）")
if w.state_n == 0:
    fails.append("没有 vehicle_state 数据流")
if w.raw_path_n == 0:
    fails.append("参考路径未被订阅方看到（velocity_profiler 输入侧断链）")
if w.speeds_n == 0:
    fails.append("pathlimits 未携带等长 target_speeds（#14 速度载体缺失，profiler→PP 断链）")
if w.speed_max < min_speed:
    fails.append("车辆未动起来：闭环实际未闭合")
if w.cmd_n <= 20:
    fails.append("控制指令流缺失或过少")
if w.cmd_bad_frame or w.cmd_bad_sum:
    fails.append("出口指令帧违反 #15 帧契约（帧头/长度/校验和）")
if w.cmd_brake_only:
    fails.append("出口始终是制动：行驶许可/速度授权未打通")
rclpy.shutdown()

if fails:
    print("CLOSED-LOOP SMOKE FAILED:")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("数据面断言通过")
PY
rc=$?
if [ "$rc" -ne 0 ]; then
    echo "FAIL: 数据面断言未通过；节点日志尾部："
    tail -n 3 "$OUT_DIR"/*.log
    exit 1
fi

# 先干净关停，让 track_benchmark 在析构里落盘报告（md + json）
echo "=== 关停并检查评测报告落盘 ==="
cleanup
trap - EXIT

if [ ! -f "$OUT_DIR/closed_loop_report.json" ]; then
    echo "FAIL: 未生成 $OUT_DIR/closed_loop_report.json（track_benchmark 报告未接入闭环）"
    tail -n 5 "$OUT_DIR/benchmark.log"
    exit 1
fi
samples="$(grep -oE '"total_samples": [0-9]+' "$OUT_DIR/closed_loop_report.json" | grep -oE '[0-9]+')"
status="$(grep -oE '"run_status": "[^"]+"' "$OUT_DIR/closed_loop_report.json" | head -1)"
echo "OK   报告存在：${status:-<no run_status>} total_samples=${samples:-0}"
if [ -z "${samples:-0}" ] || [ "${samples:-0}" -le 0 ]; then
    echo "FAIL: 闭环评测样本数为 0（评测器没有吃到仿真状态）"
    exit 1
fi
echo "Closed-loop simulation smoke: PASS ✔  （容差型集成门禁，不要求位级一致）"
