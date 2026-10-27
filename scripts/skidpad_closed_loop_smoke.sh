#!/usr/bin/env bash
# ==============================================================================
# #92：八字绕环（skidpad）**在线规划器闭环**数据面门禁。
#
# 与 skidpad_finish_smoke.sh（#91）的分工：
#   - #91 是真值中心线直推完赛门禁（证明仲裁器 + 仿真器 + 判据核完赛收敛）；
#   - 本脚本是在线规划器数据面门禁（证明感知锥桶图 → skidpad_planner 在线规划
#     → velocity_profiler 速度剖面 → pure_pursuit 控制 → arbiter 仲裁 → simulator
#     整个在线链路的数据流动性、规划器产出率与车辆推进能力）。
#   - 完赛（finished + REQUEST 停）为本单非目标（记录为 JSON 观察字段）。
#
# 拓扑（在线规划链）：
#   vehicle_simulator(/sensors/cones/fused + /localization/vehicle_state)
#     → skidpad_planner(output_pathlimits_topic 重映射到 /planning/raw_pathlimits)
#     → velocity_profiler(→ /planning/pathlimits)
#     → pure_pursuit(→ /control/vehicle_command)
#     → command_arbiter_node(→ /vehicle_command)
#     → vehicle_simulator(物理积分回流)
#   safety_monitor → /system/stop
#
# 数据面断言：
#   1) 全部节点在预算内注册（ros2 daemon 口径）；
#   2) 规划器以 ≥1 Hz 产出非空 pathlimits，字段全部有限（无 NaN/Inf）；
#   3) 车辆沿八字推进：对右圆圆心的绕角累积 ≥ 2π（用 tracks.json 元数据）；
#   4) 横向走廊：全程对参考中心线最大横向偏差 ≤ 1.0 m；
#   5) 指令流契约：出口指令帧满足 #15 帧头与校验和契约。
#
# 负样本可证伪：
#   ENABLE_PLANNER=0：不拉起 skidpad_planner → 无 pathlimits 产出 → 门禁必须退出非 0
#   且原因明确指向无 pathlimits。
#
# ── 挂载状态：**已挂载**（#92 收口，2026-10-27）────────────────────────────────
#   本脚本已挂进 scripts/drive_gates.sh 的 `skidpad-closed-loop` 门与
#   .github/workflows/ci.yml 的 "Skidpad closed-loop smoke" 步骤（两臂：正样本 +
#   ENABLE_PLANNER=0 可证伪负样本）。
#
#   挂载前提（#92 评论「六、重新进入本单的条件」）已全部满足：
#     1) #95 的几何缺陷已修复并合入 main（b4d2d1a，同心双壁瓣模型）；
#     2) 正样本实测达标：右圆绕角 6.2864 rad ≥ 2π、横向走廊 0.433 m ≤ 1.0 m、
#        pathlimits 20.04 Hz 且全有限、出口 1399 帧 0 违例、退出码 0；
#     3) 本次挂载 PR 显式说明挂载动作，且**未改动任何判据阈值**
#        （横向走廊 1.0 m、目标绕角 2π、发布率 1 Hz、帧契约均保持原值）。
#
#   「末速」口径（#92 收口时 Planner 裁定）：Issue 正文原写的「末速 < 0.1 m/s」
#   与本单「完赛/停 非目标」自相矛盾（首轮失败报告里的「末速 0.0」恰恰是**停滞**
#   症状），裁定按「笔误，实为断言**未停滞**」处理 → 脚本断言改为 **末速 ≥ 0.1 m/s**
#   （仅正样本臂；负样本臂不拉规划器，车本就静止，不适用该断言）。
#
#   手工用法：
#     bash scripts/skidpad_closed_loop_smoke.sh build/skidpad_closed_loop_smoke
#     ENABLE_PLANNER=0 bash scripts/skidpad_closed_loop_smoke.sh <out>   # 负样本
# ==============================================================================
export ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=SHM
export PATH="$HOME/.local/bin:$PATH"
: "${ROS_HOME:=$PWD/build/.ros}"; export ROS_HOME; mkdir -p "$ROS_HOME"
: "${TMPDIR:=$PWD/build/tmp}"; export TMPDIR; mkdir -p "$TMPDIR"

set +u
[ -f /opt/ros/lyrical/setup.bash ] && source /opt/ros/lyrical/setup.bash
[ -f /opt/ros/jazzy/setup.bash ] && source /opt/ros/jazzy/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
set -uo pipefail

# #69：就绪探测共用件（进程存活为准，daemon 视图降为辅助确认）。
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/node_readiness.sh
source "${SCRIPT_DIR}/lib/node_readiness.sh"

# PREWARM daemon
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true
timeout 40 ros2 daemon start >/dev/null 2>&1 || true
sleep 3

OUT_DIR="${1:-build/skidpad_closed_loop_smoke}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json

BUDGET_SEC="${BUDGET_SEC:-60}"
ENABLE_PLANNER="${ENABLE_PLANNER:-1}"
MAX_V="${MAX_V:-5.0}"
TRACK_ID="${TRACK_ID:-skidpad-figure8}"
TRACKS_DIR="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"
LATERAL_MAX="${LATERAL_MAX:-1.0}"
TARGET_ANGLE_RAD="${TARGET_ANGLE_RAD:-6.2831853}"  # 2π rad
INIT_X="${INIT_X:--15.0}"
INIT_Y="${INIT_Y:-0.0}"
INIT_THETA="${INIT_THETA:-0.0}"

TRACK_CSV="$TRACKS_DIR/skidpad_track.csv"
[ -f "$TRACK_CSV" ] || { echo "FAIL: 赛道几何缺失 $TRACK_CSV"; exit 1; }

# ── 1) 导出赛道元数据与中心线 ───────────────────────────────────────────────────
EXPORT_DIR="$OUT_DIR/tracks_export"
mkdir -p "$EXPORT_DIR"
ros2 run track_benchmark benchmark_runner --export-tracks "$EXPORT_DIR" >/dev/null 2>"$OUT_DIR/export.log"
[ -f "$EXPORT_DIR/tracks.json" ] || { echo "FAIL: 未能导出 tracks.json"; tail -5 "$OUT_DIR/export.log"; exit 1; }
[ -f "$EXPORT_DIR/skidpad_centerline.csv" ] || { echo "FAIL: 未能导出 skidpad_centerline.csv"; exit 1; }

if ! META_OUT="$(python3 - "$EXPORT_DIR/tracks.json" "$TRACK_ID" <<'PY'
import json, sys
man, tid = sys.argv[1], sys.argv[2]
t = next((x for x in json.load(open(man))["tracks"] if x["id"] == tid), None)
if t is None:
    sys.stderr.write(f"track {tid} 不在 tracks.json\n"); sys.exit(1)
m = t.get("skidpad")
if m is None:
    sys.stderr.write(f"{tid} 无 skidpad 元数据\n"); sys.exit(1)
print(f"{m['right_center_x']} {m['right_center_y']} {m['left_center_x']} {m['left_center_y']} "
      f"{m['path_radius']} {m['required_laps']} {m['exit_x']}")
PY
)"; then
    echo "FAIL: 无法从 tracks.json 取 skidpad 元数据"
    exit 1
fi
read -r RCX RCY LCX LCY RADIUS LAPS EXIT_X <<<"$META_OUT"
echo "=== skidpad 在线闭环冒烟：右圆($RCX,$RCY) 左圆($LCX,$LCY) r=$RADIUS ENABLE_PLANNER=$ENABLE_PLANNER ==="

PIDS=()
NODE_PIDS=()
cleanup() {
    local p c alive
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done
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

start() {
    local node="$1" log="$2"
    shift 2
    "$@" >"$log" 2>&1 & local pid=$!
    PIDS+=($pid)
    probe_register "$node" "$pid"
}

# ── 2) 拉起在线闭环链路 ────────────────────────────────────────────────────────
start vehicle_simulator "$OUT_DIR/sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV" \
    -p init_x:="$INIT_X" -p init_y:="$INIT_Y" -p init_theta:="$INIT_THETA"

start safety_monitor "$OUT_DIR/safety.log" ros2 run safety_monitor safety_monitor --ros-args \
    -p use_sim_time:=true

start velocity_profiler "$OUT_DIR/profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true -p limits.max_velocity:="$MAX_V"

start pure_pursuit "$OUT_DIR/pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
    -p use_sim_time:=true -p topics.vehicle_command:=/control/vehicle_command \
    -p algorithm.throttle.speed_high_threshold:=10.0

start command_arbiter_node "$OUT_DIR/arbiter.log" ros2 run safety_monitor command_arbiter_node --ros-args \
    -p use_sim_time:=true -p task.autostart:=true -p arbitration.source_timeout_sec:=0.5

EXPECT_NODES="vehicle_simulator safety_monitor velocity_profiler pure_pursuit command_arbiter_node"
if [ "$ENABLE_PLANNER" = "1" ]; then
    start skidpad_planner "$OUT_DIR/planner.log" ros2 run skidpad_planner skidpad_planner --ros-args \
        -p use_sim_time:=true -p road_type:=1 \
        -p input_cone_map_topic:=/sensors/cones/fused \
        -p output_pathlimits_topic:=/planning/raw_pathlimits \
        -p vehicle_state_topic:=/localization/vehicle_state
    EXPECT_NODES="$EXPECT_NODES skidpad_planner"
fi

# ── 就绪探测（#69：进程存活为准，daemon 视图降为辅助确认）────────────────────
# 旧实现只看 `ros2 node list` 的 daemon 视图 —— 那是**就绪探测的单点失效**
# （main 的 run 40205834746：6 个节点进程日志全部正常，daemon 视图却是空表，
# 于是门禁把"环境发现退化"误判成"节点没起来"而判红）。
# 现在：进程存活是主判据（进程退出 = 真失败，立即判红并点名）；daemon 视图
# 只作辅助确认，视图空不判红，改由数据面断言决定成败。
probe_wait_ready "$EXPECT_NODES" 12 2
probe_rc=$?
case "$probe_rc" in
    0)
        : # daemon 视图确认就绪
        ;;
    1)
        echo "FAIL: 闭环节点未全部起来（节点进程已退出 = 真失败，不是环境问题）："
        printf '%s\n' "$PROBE_DEAD_NODES" | sed 's/^/  节点没起来: /'
        tail -n 3 "$OUT_DIR"/*.log
        exit 1
        ;;
    2)
        PROBE_ENV_SUSPECT=1
        probe_env_hint
        echo "  最后一次 ros2 node list 输出："
        printf '%s\n' "$PROBE_LAST_NODE_VIEW" | sed 's/^/    /'
        ;;
esac
for p in "${PIDS[@]}"; do
    while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
done

# 节点注册表（#92 验收第 1 条要求 JSON 留痕含节点注册表）：形如
# "vehicle_simulator:1234 safety_monitor:1235 …"，随快照一起落盘。
NODE_REGISTRY=""
for i in "${!PROBE_NODE_NAMES[@]}"; do
    NODE_REGISTRY+="${PROBE_NODE_NAMES[$i]}:${PROBE_NODE_PIDS[$i]} "
done
echo "OK   $(echo "$EXPECT_NODES" | wc -w) 个节点进程已就绪，开始数据面断言"

# ── 3) 数据面观测与判据落盘 ─────────────────────────────────────────────────────
python3 - "$EXPORT_DIR/skidpad_centerline.csv" "$OUT_DIR" "$RCX" "$RCY" "$BUDGET_SEC" \
    "$LATERAL_MAX" "$TARGET_ANGLE_RAD" "$ENABLE_PLANNER" "$NODE_REGISTRY" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatCarstate, HuatPathLimits, HuatSystemState, HuatVehicleCmd
from std_msgs.msg import String

csv_path = sys.argv[1]
out_dir = sys.argv[2]
rcx, rcy = float(sys.argv[3]), float(sys.argv[4])
budget = float(sys.argv[5])
lateral_max = float(sys.argv[6])
target_angle = float(sys.argv[7])
expect_planner = sys.argv[8] == "1"
# 节点注册表（#92 验收第 1 条）：形如 "vehicle_simulator:1234 …"
node_registry = sys.argv[9].split() if len(sys.argv) > 9 else []

# 参考中心线（#90 导出，349 点）
ref_pts = []
for line in open(csv_path):
    if line.startswith("#"):
        continue
    f = line.split(",")
    ref_pts.append((float(f[0]), float(f[1])))


class DataPlaneWatch(Node):
    def __init__(self):
        super().__init__("skidpad_closed_loop_watch")
        self.raw_path_n = 0
        self.prof_path_n = 0
        self.state_n = 0
        self.cmd_n = 0
        self.cmd_bad_frame = 0
        self.cmd_bad_sum = 0
        self.pos = None
        self.v_last = 0.0
        self.v_max = 0.0
        self.max_dev = 0.0
        self.max_dev_at = None

        self.last_path_stamp = None
        self.path_finite = True
        self.path_non_empty_count = 0
        self.path_first_time = None
        self.path_last_time = None

        self.prev_angle = None
        self.cum_right_angle = 0.0
        self.finished_observed = False
        self.final_sys_state = None

        self.create_subscription(HuatPathLimits, "/planning/raw_pathlimits", self.on_raw_path, 20)
        self.create_subscription(HuatPathLimits, "/planning/pathlimits", self.on_prof_path, 20)
        self.create_subscription(HuatCarstate, "/localization/vehicle_state", self.on_state, 50)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_cmd, 50)
        self.create_subscription(HuatSystemState, "/system/state", self.on_sys, 50)
        self.create_subscription(String, "/system/task_event", self.on_event, 50)

    def on_raw_path(self, msg):
        now_t = time.time()
        self.raw_path_n += 1
        if self.path_first_time is None:
            self.path_first_time = now_t
        self.path_last_time = now_t

        if not msg.path:
            return
        self.path_non_empty_count += 1
        for p in msg.path:
            if not (math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z)):
                self.path_finite = False
        for c in msg.tracklimits.left:
            if not (math.isfinite(c.position_base_link.x) and math.isfinite(c.position_base_link.y)):
                self.path_finite = False
        for c in msg.tracklimits.right:
            if not (math.isfinite(c.position_base_link.x) and math.isfinite(c.position_base_link.y)):
                self.path_finite = False

    def on_prof_path(self, msg):
        self.prof_path_n += 1

    def on_state(self, msg):
        self.state_n += 1
        x, y, v = msg.car_state.x, msg.car_state.y, msg.v
        self.pos = (x, y)
        self.v_last = v
        self.v_max = max(self.v_max, v)

        # 绕右圆圆心累积角度
        cur_ang = math.atan2(y - rcy, x - rcx)
        if self.prev_angle is not None:
            diff = cur_ang - self.prev_angle
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            # 车沿右圆逆时针或顺时针推进时角度单调变化，累积实际绕行绝对弧度
            self.cum_right_angle += abs(diff)
        self.prev_angle = cur_ang

        # 横向走廊距离
        if ref_pts:
            dmin = min((x - px) ** 2 + (y - py) ** 2 for px, py in ref_pts)
            dev = math.sqrt(dmin)
            if dev > self.max_dev:
                self.max_dev, self.max_dev_at = dev, (round(x, 2), round(y, 2))

    def on_cmd(self, msg):
        self.cmd_n += 1
        if (msg.head1, msg.head2, msg.length) != (0xAA, 0x55, 10):
            self.cmd_bad_frame += 1
            return
        s = (msg.steering + msg.brake_force + msg.pedal_ratio + msg.gear_position +
             msg.working_mode + msg.racing_num + msg.racing_status) & 0xFFFF
        if s != msg.checksum:
            self.cmd_bad_sum += 1

    def on_sys(self, msg):
        self.final_sys_state = msg

    def on_event(self, msg):
        if msg.data.strip().lower() == "finish":
            self.finished_observed = True


rclpy.init()
watch = DataPlaneWatch()
t0 = time.time()
while time.time() - t0 < budget:
    rclpy.spin_once(watch, timeout_sec=0.1)
    # 若绕角已达成目标（≥2π），可提前结束正样本
    if expect_planner and watch.cum_right_angle >= target_angle:
        break
    # 负样本若已超时 5s 且无 path，可提前判定
    if not expect_planner and (time.time() - t0 > 5.0) and watch.raw_path_n == 0:
        break

wall_used = round(time.time() - t0, 1)
path_duration = (watch.path_last_time - watch.path_first_time) if (watch.path_first_time and watch.path_last_time) else 0.0
path_rate_hz = round(watch.raw_path_n / path_duration, 2) if path_duration > 0.5 else (round(watch.raw_path_n / wall_used, 2) if wall_used > 0 else 0.0)

snap = {
    "schema": "fsac.skidpad_closed_loop_smoke/v1",
    "enable_planner": expect_planner,
    "nodes": node_registry,
    "wall_used_s": wall_used,
    "budget_s": budget,
    "state_msgs": watch.state_n,
    "raw_path_msgs": watch.raw_path_n,
    "non_empty_path_msgs": watch.path_non_empty_count,
    "pathlimits_rate_hz": path_rate_hz,
    "pathlimits_finite": watch.path_finite,
    "profiler_forwarded_msgs": watch.prof_path_n,
    "v_max": round(watch.v_max, 3),
    "v_final": round(watch.v_last, 3),
    "cum_right_angle_rad": round(watch.cum_right_angle, 4),
    "target_angle_rad": round(target_angle, 4),
    "max_lateral_dev_m": round(watch.max_dev, 3),
    "max_lateral_dev_at": watch.max_dev_at,
    "lateral_max_allowed": lateral_max,
    "cmd_msgs": watch.cmd_n,
    "cmd_bad_frame": watch.cmd_bad_frame,
    "cmd_bad_checksum": watch.cmd_bad_sum,
    "finished_observed": watch.finished_observed,
    "final_pos": [round(watch.pos[0], 2), round(watch.pos[1], 2)] if watch.pos else None,
}

with open(f"{out_dir}/skidpad_closed_loop_report.json", "w") as f:
    json.dump(snap, f, indent=2, sort_keys=True)

print("数据面快照：")
for k, v in snap.items():
    print(f"  {k:<26}: {v}")
rclpy.shutdown()

fails = []
if expect_planner:
    # 正样本断言：
    # 1. 规划器以 ≥1 Hz 产出非空 pathlimits，字段全部有限
    if watch.raw_path_n == 0:
        fails.append("规划器未发布 /planning/raw_pathlimits（规划器断链）")
    elif path_rate_hz < 1.0:
        fails.append(f"规划器发布频率过低：{path_rate_hz} Hz < 1.0 Hz")
    if watch.path_non_empty_count == 0:
        fails.append("规划器输出路径全部为空")
    if not watch.path_finite:
        fails.append("规划器输出包含非有限值（NaN 或 Inf）")

    # 2. 闭环链路流动
    if watch.prof_path_n == 0:
        fails.append("velocity_profiler 未转发路径（速度剖面断链）")
    if watch.state_n == 0:
        fails.append("没有 vehicle_state 数据流（仿真状态未回流）")
    if watch.cmd_n <= 10:
        fails.append("控制指令流缺失或过少（控制器/仲裁器未产出指令）")
    if watch.cmd_bad_frame or watch.cmd_bad_sum:
        fails.append(f"出口指令帧违反 #15 帧契约（坏帧 {watch.cmd_bad_frame} / 坏校验和 {watch.cmd_bad_sum}）")

    # 3. 推进能力：右圆绕角 ≥ 2π
    if watch.cum_right_angle < target_angle:
        fails.append(f"车辆推进不足：右圆绕角 {watch.cum_right_angle:.3f} rad < 目标 {target_angle:.3f} rad (2π)"
                     f"（末位姿 {snap['final_pos']}，末速 {watch.v_last:.3f} m/s）")

    # 4. 横向走廊 ≤ 1.0m
    if watch.max_dev > lateral_max:
        fails.append(f"横向走廊超限：max_lateral_dev={watch.max_dev:.3f} m > 阈值 {lateral_max} m（在 {watch.max_dev_at}）")

    # 5. 未停滞：末速 ≥ 0.1 m/s
    #    Issue #92 正文原写的「末速 < 0.1 m/s」与本单「完赛/停 非目标」自相矛盾
    #    （首轮失败报告里的「末速 0.0」恰恰是**停滞**症状），#92 收口时 Planner 裁定
    #    按「笔误，实为断言未停滞」处理 → 此处断言车仍在走（≥0.1 m/s），
    #    否则「绕角达标」可能只是原地打转凑出来的。
    #    仅正样本臂适用：负样本臂不拉规划器，车本就静止。
    if watch.v_last < 0.1:
        fails.append(f"车辆停滞：末速 {watch.v_last:.3f} m/s < 0.1 m/s"
                     f"（绕角 {watch.cum_right_angle:.3f} rad / 末位姿 {snap['final_pos']}）")
else:
    # 负样本断言（ENABLE_PLANNER=0）：必须无 pathlimits 产出且门禁失败
    if watch.raw_path_n > 0:
        fails.append(f"负样本已禁用规划器却仍收到 {watch.raw_path_n} 条路径消息（开关失效）")
    else:
        fails.append("负样本验证通过：无 skidpad_planner 时无 pathlimits 产出（数据面链条可证伪）")

if fails:
    print("SKIDPAD-CLOSED-LOOP SMOKE ASSERTION RESULTS:")
    for f in fails:
        print("  -", f)
    # 若负样本且唯一失败项是预期的证伪项，则按负样本可证伪逻辑退出非 0
    sys.exit(1)

print("数据面断言全部通过 ✔")
PY
rc=$?
if [ "$rc" -ne 0 ]; then
    # #69：环境性失败与真失败必须可区分 —— 首行提示语。
    if [ "${PROBE_ENV_SUSPECT:-0}" = "1" ]; then
        echo "疑环境性失败（DDS 发现未收敛）——请核对 CI 同 commit 结论后再改代码"
    fi
    echo "SKIDPAD CLOSED LOOP SMOKE EXITED WITH CODE $rc"
    exit $rc
fi

echo "Skidpad closed-loop smoke: PASS ✔"
