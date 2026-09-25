#!/usr/bin/env bash
# ==============================================================================
# #17：闭环故障样本冒烟 —— 在真实 ROS 多节点回路里注入 越界/反向/未完赛/超时，
# 断言闭环内的 track_benchmark(KpiEvaluator) 报告**真的命中对应失败判据**，而不是只在
# 单测里证明判定逻辑（对齐 #17 验收：“人工构造…评测均能正确识别”）。
#
# 与 closed_loop_sim_smoke.sh 分工：那个证“回路是否闭合/数据在流动”（只跑正常相）；本脚本
# 证“失败可被识别”（正常相作阳性对照 + 四类故障相）。
#
# 回路：vehicle_simulator(物理 + /clock)
#         → 参考路径喂入 /planning/raw_pathlimits
#         → velocity_profiler(→ /planning/pathlimits 带 target_speeds)
#         → pure_pursuit(→ /vehicle_command) → vehicle_simulator
#       track_benchmark 订阅 state+command，关停时落终态 KPI JSON。
#
# 参考路径**直接取自 track_benchmark 锁存发布的真实赛道中心线**(/benchmark/centerline_path)，
# 与评测器同源（单一赛道来源），不另造几何。车初始位姿对齐中心线首点（见下方 START_*）。
#
# 场景与判据（读 $OUT_DIR/<name>.json）：
#   nominal  正向跟完 1 圈   → run_status=finished 且 valid_laps>=1 且 out_of_bounds_events=0
#   offroute 横向渐变偏置路径 → out_of_bounds_events>0
#   reverse  反向路径顺序     → net_arc_progress_m<0 且 valid_laps=0 且 best_valid_lap_time_s=0
#   dnf      只喂最前一段路径 → run_status=incomplete 且 valid_laps=0 且 best_valid_lap_time_s=0
#   timeout  限时 5s 跑全圈   → run_status=timeout 且 timed_out=true
#
# 判据采用容差（DDS/调度天然抖动），非位级。始终在墙钟预算内自动退出；进程清理只按 PID
# 递进 INT→TERM→KILL（Jazzy 的 ros2 run wrapper 不转发信号）。
# 本机注意：与既有门禁一致，多节点前需 export FASTDDS_BUILTIN_TRANSPORTS=SHM（CI 上不需要）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"

OUT_DIR="${1:-build/closed_loop_fault}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json "$OUT_DIR"/*.md

# 赛道中心线首点位姿（trackdrive-loop/v1 参数化几何在 t=0 的取值；几何由 track_generator
# 生成、并由单测 TrackRegistry.GeometryChecksumIsFixedForCommittedVersion 锁定。若赛道几何
# 变更，该单测会先失败，届时同步更新此常量与基线）。
START_X=0.0
START_Y=20.0
START_THETA=0.224453
REV_THETA="$(python3 -c 'import math; print(0.224453 + math.pi)')"  # 反向相：车头朝反方向

# 场景表：name|仿真时长(s)|墙钟上限(s)|初始航向|最大运行时长(s)|横向偏置(m)|要求圈数
# 用**仿真时长**而非墙钟来定喂入长度：CI runner 上 100Hz 物理可能跑不满实时，若按墙钟计时
# 会在车还没跑完一圈时就收工，把 nominal 误判为未完赛。墙钟上限只作看门狗兜底。
SCENARIOS=(
    "nominal|55|150|${START_THETA}|0.0|0|1"
    "offroute|30|100|${START_THETA}|0.0|4.0|1"
    "reverse|30|100|${REV_THETA}|0.0|0|1"
    "dnf|30|100|${START_THETA}|0.0|0|1"
    "timeout|15|60|${START_THETA}|5.0|0|1"
)

TRACK_CSV="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}/trackdrive_loop.csv"
if [ ! -f "$TRACK_CSV" ]; then
    echo "FAIL: 赛道几何缺失 $TRACK_CSV（应由 benchmark_regression.sh 的导出步骤保证）"
    exit 1
fi

PIDS=()       # ros2 run wrapper
NODE_PIDS=()  # wrapper 的直接子进程 = 真实节点

cleanup() {
    local p c alive
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done
    # 先对节点发 INT：让 track_benchmark 在析构里落终态报告（顺序很重要）
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
    wait 2>/dev/null || true
}
trap cleanup EXIT

start() {  # start <logfile> <cmd...>
    local log="$1"
    shift
    "$@" >"$log" 2>&1 &
    PIDS+=($!)
}

# 参考路径喂入节点：订阅锁存的 /benchmark/centerline_path，按场景变换后发到
# /planning/raw_pathlimits。写成脚本内派生文件，避免再引一份工作区外的脚本。
cat >"$OUT_DIR/feeder.py" <<'PY'
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from common_msgs.msg import HuatPathLimits
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock

# out_dir, scenario, sim_dur, wall_cap, offset_m
out_dir, scenario, sim_dur, wall_cap, offset_m = (
    sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), float(sys.argv[5]))


class Feeder(Node):
    def __init__(self):
        super().__init__("closed_loop_fault_feeder")
        self.pts = None
        self.clock_t = None  # 最后一次 /clock 的仿真时刻
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, "/benchmark/centerline_path", self.on_cl, qos)
        self.create_subscription(Clock, "/clock", self.on_clock, 20)
        self.pub = self.create_publisher(HuatPathLimits, "/planning/raw_pathlimits", 10)
        self.create_timer(0.1, self.tick)

    def on_clock(self, m):
        self.clock_t = m.clock.sec + m.clock.nanosec * 1e-9

    def on_cl(self, m):
        self.pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def transformed(self):
        pts = self.pts
        if scenario == "reverse":
            # PP 只在数组里“向前”找前视点：直接发 reversed 会让车的最近点落在末元素、前方
            # 无点而拒跟。故旋转使 p0 仍在首位、其后接逆序 → 前视方向变为沿赛道倒退。
            pts = [pts[0]] + list(reversed(pts[1:]))
        elif scenario == "dnf":
            pts = pts[: max(2, len(pts) // 5)]
        elif scenario == "offroute" and offset_m != 0.0:
            # 恒定位移会被 PP 当飞点拒跟（车原地不动）；沿路径把位移从 0 渐变到 offset_m，
            # 车先对齐再被逐步带出合法走廊。
            out = []
            n = len(pts)
            ramp = max(1, n // 7)
            for i, (x, y) in enumerate(pts):
                a, b = pts[(i - 1) % n], pts[(i + 1) % n]
                tx, ty = b[0] - a[0], b[1] - a[1]
                norm = math.hypot(tx, ty) or 1.0
                k = offset_m * min(1.0, i / ramp)
                out.append((x - ty / norm * k, y + tx / norm * k))
            pts = out
        return pts

    def tick(self):
        if not self.pts:
            return
        m = HuatPathLimits()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.path = [Point(x=x, y=y, z=0.0) for x, y in self.transformed()]
        m.replan = False
        self.pub.publish(m)


rclpy.init()
w = Feeder()
t0 = time.time()
clock0 = None
while True:
    rclpy.spin_once(w, timeout_sec=0.1)
    wall = time.time() - t0
    if w.clock_t is not None:
        if clock0 is None:
            clock0 = w.clock_t
        # 主判据：仿真时间推进到目标时长（与 CI runner 快慢无关）
        if w.clock_t - clock0 >= sim_dur:
            break
    if w.pts is None and wall > 12:
        print("feeder: 未收到 /benchmark/centerline_path", file=sys.stderr)
        break
    if wall >= wall_cap:  # 看门狗：仿真时钟没推进/过慢也不至于挂死
        print(f"feeder: 墙钟上限 {wall_cap}s 到达（仿真仅推进 {(w.clock_t - clock0) if clock0 else 0:.1f}s）", file=sys.stderr)
        break
print(f"feeder: scenario={scenario} pts={len(w.pts) if w.pts else 0}", file=sys.stderr)
rclpy.shutdown()
PY

# ROS 环境：调用方已 source 就沿用，否则尽力自行 source（须临时关 nounset，见 drive_gates.sh）。
source_sh() {
    local f="$1"
    [ -f "${f}" ] || return 0
    set +u
    # shellcheck disable=SC1090
    source "${f}" 2>/dev/null || true
    set -u
}
if ! command -v ros2 >/dev/null 2>&1; then
    for cand in /opt/ros/*/setup.bash; do
        [ -f "${cand}" ] && { source_sh "${cand}"; break; }
    done
fi
source_sh install/setup.bash

required_nodes=(vehicle_simulator safety_monitor velocity_profiler pure_pursuit track_benchmark_node)

echo "=== 闭环故障冒烟：${#SCENARIOS[@]} 个场景（正常对照 + 越界/反向/未完赛/超时） ==="
for spec in "${SCENARIOS[@]}"; do
    IFS='|' read -r name sim_dur wall_cap theta maxrt offset require_laps <<<"$spec"
    echo "--- 场景 $name（仿真 ${sim_dur}s / 墙钟上限 ${wall_cap}s，max_runtime=${maxrt}s，横向偏置 ${offset}m） ---"

    PIDS=()
    NODE_PIDS=()
    start "$OUT_DIR/${name}_sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
        -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV" \
        -p init_x:="${START_X}" -p init_y:="${START_Y}" -p init_theta:="${theta}"
    start "$OUT_DIR/${name}_safety.log" ros2 run safety_monitor safety_monitor --ros-args -p use_sim_time:=true
    start "$OUT_DIR/${name}_profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
        -p use_sim_time:=true
    # PP 直接发到契约出口 /vehicle_command（本冒烟不拉起仲裁器，避免与 sim 回路双发布者）
    start "$OUT_DIR/${name}_pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
        -p use_sim_time:=true -p topics.vehicle_command:=/vehicle_command
    start "$OUT_DIR/${name}_bench.log" ros2 run track_benchmark track_benchmark_node --ros-args \
        -p use_sim_time:=true -p track_type:=trackdrive -p controller_name:=PurePursuit \
        -p require_laps:="${require_laps}" -p max_runtime_s:="${maxrt}" \
        -p report_file:="$OUT_DIR/${name}.md"

    # 等节点上图（最多 ~15 轮，每轮自带超时）；用 --no-daemon 直接查图，避免 daemon 空表误判。
    ready=0
    nodes=""
    for _ in $(seq 1 15); do
        nodes="$(timeout 12 ros2 node list --no-daemon 2>/dev/null)"
        missing=0
        for n in "${required_nodes[@]}"; do
            grep -q "$n" <<<"$nodes" || missing=1
        done
        [ "$missing" -eq 0 ] && { ready=1; break; }
        sleep 2
    done
    if [ "$ready" -ne 1 ]; then
        echo "FAIL $name: 闭环节点未全部注册；最后一次 ros2 node list："
        printf '%s\n' "$nodes" | sed 's/^/  /'
        echo "缺失判据节点：${required_nodes[*]}"
        tail -n 3 "$OUT_DIR"/${name}_*.log
        cleanup
        continue
    fi
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done

    python3 "$OUT_DIR/feeder.py" "$OUT_DIR" "$name" "$sim_dur" "$wall_cap" "$offset"
    cleanup   # 关停让 track_benchmark 析构落终态报告；同时清掉本轮全部 PID
done
trap - EXIT

echo "=== 判据断言（闭环内评测器是否认出失败） ==="
python3 - "$OUT_DIR" <<'PY'
import json, os, sys

out_dir = sys.argv[1]


def load(name):
    p = os.path.join(out_dir, name + ".json")
    return json.load(open(p)) if os.path.isfile(p) else None


def check(d):
    if d is None:
        return False, "报告缺失"
    if not d.get("total_samples", 0):
        return False, "评测器没有吃到仿真状态"
    return d, None


# 场景: 判据函数（返回 (ok, 说明)）
def nominal(d):
    ok = d["run_status"] == "finished" and d["valid_laps"] >= 1 and d["out_of_bounds_events"] == 0
    return ok, f"run_status={d['run_status']} valid_laps={d['valid_laps']} oob={d['out_of_bounds_events']}"


def offroute(d):
    ok = d["out_of_bounds_events"] > 0 and d["out_of_bounds_samples"] > 0
    return ok, f"out_of_bounds_events={d['out_of_bounds_events']} samples={d['out_of_bounds_samples']}"


def reverse(d):
    ok = d["net_arc_progress_m"] < 0 and d["valid_laps"] == 0 and d["best_valid_lap_time_s"] == 0
    return ok, f"net_arc_progress_m={d['net_arc_progress_m']} valid_laps={d['valid_laps']}"


def dnf(d):
    ok = d["run_status"] == "incomplete" and d["valid_laps"] == 0 and d["best_valid_lap_time_s"] == 0
    return ok, f"run_status={d['run_status']} valid_laps={d['valid_laps']} best_lap={d['best_valid_lap_time_s']}"


def timeout(d):
    ok = d["run_status"] == "timeout" and d["timed_out"] is True
    return ok, f"run_status={d['run_status']} timed_out={d['timed_out']}"


checks = [("nominal", nominal), ("offroute", offroute), ("reverse", reverse), ("dnf", dnf), ("timeout", timeout)]
fails = []
for name, fn in checks:
    d, err = check(load(name))
    if err:
        fails.append(name)
        print(f"FAIL {name}: {err}")
        continue
    ok, detail = fn(d)
    print(f"{'OK  ' if ok else 'FAIL'} {name}: {detail}")
    if not ok:
        fails.append(name)

if fails:
    print("闭环故障冒烟 FAILED:", ", ".join(fails))
    sys.exit(1)
print("闭环故障冒烟 PASS ✔  （越界/反向/未完赛/超时 均在闭环内被 KpiEvaluator 识别）")
PY
rc=$?
if [ "$rc" -ne 0 ]; then
    echo "FAIL: 闭环故障判据未全部命中；各场景日志尾部："
    tail -n 3 "$OUT_DIR"/*.log 2>/dev/null
    exit 1
fi
