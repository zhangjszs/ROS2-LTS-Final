#!/usr/bin/env bash
# ==============================================================================
# #82：直线加速 **ROS 闭环**完赛门禁（多节点 + 终点事件 + 锁存停 + 缓冲区停稳）。
#
# 与 closed_loop_sim_smoke.sh 的分工：那个脚本证明 trackdrive 回路"数据在流、车在动、
# 指令合法"；本脚本证明真实话题链能**按赛制过线并停进缓冲区**（D-014 首个端到端赛制）。
#
# 拓扑（仲裁器链，单一权威出口）：
#   pathsource(内联发布 /planning/raw_pathlimits，>runout_end_m 的确定性直线)
#     → velocity_profiler(→ /planning/pathlimits，速度上限=赛道元数据反推的到达速度 cap)
#     → pure_pursuit(→ /control/vehicle_command，源 A)
#     → command_arbiter_node(→ /vehicle_command，唯一出口；消费 /system/task_event)
#     → vehicle_simulator(物理积分 → /localization/vehicle_state → 回到 pathsource 视野)
#   safety_monitor → /system/stop（可恢复 TIMEOUT 停）→ 仲裁器
#   finish_event_node：订阅 vehicle_state，首次越线**恰好一次**发 finish（#80 判据核）
#
# 为什么必须拉起 command_arbiter（与 closed_loop_sim_smoke 的拓扑差异）：
#   契约 docs/INTERFACE_CONTRACT.md 规定 `finish → 锁存 REQUEST 停` 只在仲裁器的
#   TaskSafetyStateMachine 里发生；PP 直发出口时没有事件消费者，过线不会停车。
#
# 为什么断言 stop_kind==REQUEST 而不是只看"车停了"：
#   safety_monitor 的看门狗会发 /system/stop → TIMEOUT 停，与 finish 的 REQUEST 停
#   是不同来源（STOPKIND_TIMEOUT=1 / STOPKIND_REQUEST=2）。只有 REQUEST 才证明
#   **是终点事件让车停的**，而不是看门狗超时——这是本单判据与"顺带停了"的分界。
#
# 可证伪（#82 验收第 2 条）：ENABLE_FINISH_EVENT=0 时**不拉起终点事件源**，车不会因
#   过线而停车 → 必然越过缓冲区末端 → 门禁必须失败，且失败原因明确指向"未停车/越过缓冲区"。
#
# 判据阈值一律来自赛道元数据（#81 的 tracks.json），不在本脚本硬编码 75 / 100。
#
# 本机注意：与既有 runtime 门禁一样需要可写 TMPDIR（只读 /tmp 会让 ros2cli daemon
#   起不来 → `ros2 node list` 恒空 → 全部节点被误判"未注册"）。此外本机必须
#   **显式 `ros2 daemon start`**：见下方 PREWARM 段的实测说明。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"
: "${TMPDIR:=$PWD/build/tmp}"
export TMPDIR
mkdir -p "$TMPDIR"

# PREWARM：先停残留 daemon，再**显式启动**并等待上图。
# 只 `daemon stop` 指望后续调用自动重建，在"每次调用都新建进程/会话"的执行器沙箱里
# 不可靠（daemon 随父进程退出），`ros2 node list` 会一直超时返回空表——本机实测差异：
#   仅 stop → 查询 rc=124(超时)、空表；显式 start + sleep → 同沙箱内 node list 正常、
#   topic echo 能吃到 HuatCarstate。CI 上无旧 daemon，start 等同于后续调用自动创建，无副作用。
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true
timeout 40 ros2 daemon start >/dev/null 2>&1 || true
sleep 3

OUT_DIR="${1:-build/accel_finish_smoke}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json

# ── 可调项（判据阈值来自元数据，这里只调预算与开关）────────────────────────────
BUDGET_SEC="${BUDGET_SEC:-75}"            # 数据面采样墙钟上限
ENABLE_FINISH_EVENT="${ENABLE_FINISH_EVENT:-1}"   # 0 = 可证伪负样本：不拉起终点事件源
TRACK_ID="${TRACK_ID:-acceleration-75m}"
TRACKS_DIR="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"
# #80 判据核的物理常数（与 finish_judge.hpp 的 FinishRunoutSpec 缺省值一致）
BRAKE_DECEL="${BRAKE_DECEL:-9.0}"
SAFETY="${SAFETY:-0.75}"
STOP_EPS="${STOP_EPS:-0.05}"

TRACK_CSV="$TRACKS_DIR/acceleration_track.csv"
[ -f "$TRACK_CSV" ] || { echo "FAIL: 赛道几何缺失 $TRACK_CSV"; exit 1; }

# ── 1) 终点/缓冲区元数据来自 tracks.json（#81），不硬编码 ─────────────────────────
EXPORT_DIR="$OUT_DIR/tracks_export"
mkdir -p "$EXPORT_DIR"
ros2 run track_benchmark benchmark_runner --export-tracks "$EXPORT_DIR" >/dev/null 2>"$OUT_DIR/export.log"
[ -f "$EXPORT_DIR/tracks.json" ] || { echo "FAIL: 未能导出 tracks.json"; tail -5 "$OUT_DIR/export.log"; exit 1; }

if ! META_OUT="$(python3 - "$EXPORT_DIR/tracks.json" "$TRACK_ID" "$BRAKE_DECEL" "$SAFETY" <<'PY'
import json, math, sys
man, tid, decel, safety = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
tracks = json.load(open(man))["tracks"]
t = next((x for x in tracks if x["id"] == tid), None)
if t is None:
    sys.stderr.write(f"track {tid} 不在 tracks.json\n"); sys.exit(1)
if t.get("finish_x") is None or t.get("runout_end_m") is None:
    sys.stderr.write(f"{tid} 无终点元数据（finish_x/runout_end_m 为 null），本门禁不适用\n"); sys.exit(1)
fx, rm = float(t["finish_x"]), float(t["runout_end_m"])
if rm <= fx:
    sys.stderr.write(f"非法缓冲区：runout_end_m={rm} <= finish_x={fx}\n"); sys.exit(1)
# 与 #80 approachSpeedCap() 同一公式：v²/(2a) ≤ 缓冲距离 → v_cap = sqrt(2·a·runout·safety)
cap = math.sqrt(2.0 * decel * (rm - fx) * safety)
print(f"{fx} {rm} {cap}")
PY
)"; then
    echo "FAIL: 无法从 tracks.json 取终点元数据（见上方 python 报错）"
    exit 1
fi
read -r FINISH_X RUNOUT_END CAP <<<"$META_OUT"
if [ -z "${FINISH_X:-}" ] || [ -z "${RUNOUT_END:-}" ] || [ -z "${CAP:-}" ]; then
    echo "FAIL: 元数据解析结果不完整：[$META_OUT]"
    exit 1
fi
echo "=== 元数据（#81）：finish_x=$FINISH_X runout_end_m=$RUNOUT_END → 到达速度上限 cap=${CAP} m/s ==="

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
    local log="$1"
    shift
    "$@" >"$log" 2>&1 & PIDS+=($!)
}

# ── 2) 拉起闭环（速度上限取 cap：让"能停进缓冲区"成为物理可达的要求，而非事后调参）──
echo "=== 拉起仲裁器闭环（墙钟预算 ${BUDGET_SEC}s，ENABLE_FINISH_EVENT=$ENABLE_FINISH_EVENT） ==="
start "$OUT_DIR/sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV"
start "$OUT_DIR/safety.log" ros2 run safety_monitor safety_monitor --ros-args -p use_sim_time:=true
start "$OUT_DIR/profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true -p limits.max_velocity:="$CAP"
start "$OUT_DIR/pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
    -p use_sim_time:=true -p topics.vehicle_command:=/control/vehicle_command
start "$OUT_DIR/arbiter.log" ros2 run safety_monitor command_arbiter_node --ros-args \
    -p use_sim_time:=true -p task.autostart:=true -p arbitration.source_timeout_sec:=0.5

EXPECT_NODES="vehicle_simulator safety_monitor velocity_profiler pure_pursuit command_arbiter_node"
if [ "$ENABLE_FINISH_EVENT" = "1" ]; then
    start "$OUT_DIR/finish.log" ros2 run track_benchmark finish_event_node --ros-args \
        -p use_sim_time:=true -p finish.enabled:=true \
        -p finish.finish_x:="$FINISH_X" -p finish.runout_end_m:="$RUNOUT_END" \
        -p finish.brake_decel:="$BRAKE_DECEL" -p finish.stop_speed_eps:="$STOP_EPS"
    EXPECT_NODES="$EXPECT_NODES finish_event_node"
fi

ready=0
nodes=""
for _ in $(seq 1 12); do
    nodes="$(timeout 12 ros2 node list 2>/dev/null)"
    missing=0
    for n in $EXPECT_NODES; do
        grep -q "$n" <<<"$nodes" || missing=1
    done
    [ "$missing" -eq 0 ] && { ready=1; break; }
    sleep 2
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: 闭环节点未全部注册；最后一次 ros2 node list 输出："
    printf '%s\n' "$nodes" | sed 's/^/  /'
    echo "缺失判据节点：$EXPECT_NODES"
    tail -n 3 "$OUT_DIR"/*.log
    exit 1
fi
for p in "${PIDS[@]}"; do
    while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
done
echo "OK   $(echo "$EXPECT_NODES" | wc -w) 个节点已注册，开始数据面断言"

# ── 3) 数据面：内联 rclpy 既发布确定性参考路径，也观测并落盘判据 JSON ─────────────
python3 - "$OUT_DIR" "$FINISH_X" "$RUNOUT_END" "$CAP" "$BUDGET_SEC" "$STOP_EPS" "$ENABLE_FINISH_EVENT" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatCarstate, HuatPathLimits, HuatSystemState, HuatVehicleCmd
from geometry_msgs.msg import Point
from std_msgs.msg import String

out_dir = sys.argv[1]
finish_x, runout_end, cap = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
budget, stop_eps = float(sys.argv[5]), float(sys.argv[6])
expect_finish = sys.argv[7] == "1"


class Watch(Node):
    """发布覆盖到缓冲区末端之后的直线路径，并观测完赛判据。"""

    def __init__(self):
        super().__init__("accel_finish_watch")
        self.x_max = 0.0
        self.y_abs_max = 0.0
        self.state_n = 0
        self.v_last = 0.0
        self.x_last = 0.0
        self.stopped_at = None      # 首次「越线后 v<eps」的位置
        self.cmd_n = 0
        self.cmd_bad_frame = 0
        self.cmd_bad_sum = 0
        self.finish_events = []     # [(x_at_event, t_sim)]
        self.task_event_n = 0
        self.final_state = None
        self.crossed = False
        self.raw_path_n = 0

        # 参考路径：0.5 m 间距，长度覆盖 runout_end + 20 m 余量，
        # 使"车一直往前冲"在负样本里是真实可能（路径短了会因走完而停，污染判据）。
        n_pts = int(math.ceil((runout_end + 20.0) / 0.5))
        pub = self.create_publisher(HuatPathLimits, "/planning/raw_pathlimits", 10)

        def tick():
            m = HuatPathLimits()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.path = [Point(x=0.5 * i, y=0.0, z=0.0) for i in range(n_pts)]
            m.replan = False
            pub.publish(m)

        self.create_timer(0.1, tick)

        self.create_subscription(HuatCarstate, "/localization/vehicle_state", self.on_state, 50)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_cmd, 50)
        self.create_subscription(HuatSystemState, "/system/state", self.on_sys, 50)
        self.create_subscription(String, "/system/task_event", self.on_event, 50)
        self.create_subscription(HuatPathLimits, "/planning/raw_pathlimits",
                                 lambda m: setattr(self, "raw_path_n", self.raw_path_n + 1), 10)

    def on_state(self, m):
        self.state_n += 1
        x, v = m.car_state.x, m.v
        self.x_last, self.v_last = x, v
        self.x_max = max(self.x_max, x)
        self.y_abs_max = max(self.y_abs_max, abs(m.car_state.y))
        if x >= finish_x:
            self.crossed = True
        # 停稳位置：必须先越线，且速度落到阈值内
        if self.crossed and self.stopped_at is None and v < stop_eps:
            self.stopped_at = x

    def on_cmd(self, m):
        self.cmd_n += 1
        if (m.head1, m.head2, m.length) != (0xAA, 0x55, 10):
            self.cmd_bad_frame += 1
            return
        s = (m.steering + m.brake_force + m.pedal_ratio + m.gear_position +
             m.working_mode + m.racing_num + m.racing_status) & 0xFFFF
        if s != m.checksum:
            self.cmd_bad_sum += 1

    def on_sys(self, m):
        self.final_state = m

    def on_event(self, m):
        self.task_event_n += 1
        if m.data.strip().lower() == "finish":
            self.finish_events.append((self.x_last, self.state_n))


rclpy.init()
w = Watch()
t0 = time.time()
while time.time() - t0 < budget:
    rclpy.spin_once(w, timeout_sec=0.1)
    if expect_finish:
        # 正样本提前退出：越线 + 停稳 + 拿到 REQUEST 锁存态
        if w.stopped_at is not None and w.final_state is not None \
                and w.final_state.stop_kind == w.final_state.STOPKIND_REQUEST:
            break
    elif w.x_max > runout_end:
        # 负样本提前退出：越过缓冲区这一失败征象已确定，无需跑满预算
        break

snap = {
    "enable_finish_event": expect_finish,
    "finish_x": finish_x,
    "runout_end_m": runout_end,
    "speed_cap_mps": cap,
    "state_msgs": w.state_n,
    "raw_path_msgs": w.raw_path_n,
    "x_max": w.x_max,
    "y_abs_max": w.y_abs_max,
    "crossed": w.crossed,
    "finish_event_count": len(w.finish_events),
    "finish_at_x": [round(a, 3) for a, _ in w.finish_events],
    "stopped": w.stopped_at is not None,
    "stop_position_m": w.stopped_at if w.stopped_at is not None else 0.0,
    "stop_distance_m": (w.stopped_at - finish_x) if w.stopped_at is not None else 0.0,
    "v_final": w.v_last,
    "cmd_msgs": w.cmd_n,
    "cmd_bad_frame": w.cmd_bad_frame,
    "cmd_bad_checksum": w.cmd_bad_sum,
    "runout_exceeded": w.x_max > runout_end,
    "task_state": int(w.final_state.task_state) if w.final_state else -1,
    "stop_kind": int(w.final_state.stop_kind) if w.final_state else -1,
    "stop_active": bool(w.final_state.stop_active) if w.final_state else False,
    "can_drive": bool(w.final_state.can_drive) if w.final_state else True,
    "wall_used_s": round(time.time() - t0, 1),
}
with open(f"{out_dir}/accel_finish_report.json", "w") as f:
    json.dump(snap, f, indent=2, sort_keys=True)
print("判据快照：")
for k, v in snap.items():
    print(f"  {k:<20}: {v}")
rclpy.shutdown()

fails = []
if w.raw_path_n == 0:
    fails.append("参考路径未被 velocity_profiler 看到（输入侧断链）")
if w.state_n == 0:
    fails.append("没有 vehicle_state 数据流（闭环未闭合）")
if w.cmd_n <= 20:
    fails.append("控制指令流缺失或过少（仲裁器出口未工作）")
if w.cmd_bad_frame or w.cmd_bad_sum:
    fails.append(f"出口指令帧违反 #15 帧契约（坏帧 {w.cmd_bad_frame} / 坏校验和 {w.cmd_bad_sum}）")

if expect_finish:
    # 正样本：过线 + 恰好一次 + 不早发 + REQUEST 锁存 + 缓冲区内停稳 + 不出界
    if len(w.finish_events) != 1:
        fails.append(f"finish 事件应为恰好 1 次，实测 {len(w.finish_events)} 次"
                     "（0=终点事件源未触发；>1=不满足『恰好一次』）")
    elif w.finish_events[0][0] < finish_x:
        fails.append(f"finish 早发：事件时刻 x={w.finish_events[0][0]:.3f} < finish_x={finish_x}")
    if not w.crossed:
        fails.append(f"车辆未越过终点线（x_max={w.x_max:.3f} < finish_x={finish_x}）")
    if w.final_state is None:
        fails.append("未看到 /system/state（仲裁器遥测断链，锁存无法判定）")
    else:
        if w.final_state.stop_kind != w.final_state.STOPKIND_REQUEST:
            fails.append(f"停车来源不是 finish 的 REQUEST 锁存（实测 stop_kind={int(w.final_state.stop_kind)}，"
                         "1=看门狗 TIMEOUT 而非终点事件）")
        if not w.final_state.stop_active:
            fails.append("stop_active=false：锁存停未生效")
        if w.final_state.can_drive:
            fails.append("can_drive=true：锁存后仍被授予行驶许可")
    if not w.stopped_at:
        fails.append(f"未停稳：末速度 v={w.v_last:.3f} 未落到 eps={stop_eps} 之内")
    elif w.stopped_at > runout_end:
        fails.append(f"停在了缓冲区之外：stop_position_m={w.stopped_at:.3f} > runout_end_m={runout_end}")
    if w.x_max > runout_end:
        fails.append(f"曾越过缓冲区末端：x_max={w.x_max:.3f} > runout_end={runout_end}")
else:
    # 负样本（ENABLE_FINISH_EVENT=0）：缺少终点事件时本门禁**必须非 0 退出**（#82 验收第 2 条）。
    # 注意方向：这里的每一条都是“失败原因”，不是“负样本征象已命中”的确认——
    # 把征象当通过会让负样本 rc=0，门禁就失去可证伪性（本轮实测踩过这个坑）。
    if w.x_max > runout_end:
        fails.append(f"未停车 / 越过缓冲区：终点事件源已禁用（finish 事件 {len(w.finish_events)} 次）→ 仲裁器未进入锁存停"
                     f"→ 车辆冲到 x_max={w.x_max:.3f} > runout_end_m={runout_end:.3f}")
    if len(w.finish_events) != 0:
        fails.append(f"终点事件源已被禁用却仍收到 {len(w.finish_events)} 次 finish（禁用开关无效）")
    if w.stopped_at is not None and w.stopped_at <= runout_end and len(w.finish_events) == 0:
        # 没有终点事件却在缓冲区内停稳 → 停车不依赖 finish，判据存在后门（或看门狗代劳）
        fails.append(f"无 finish 事件却在缓冲区内停稳（stop_position_m={w.stopped_at:.3f}）："
                     "停车不依赖终点事件，说明存在瞬移/置零后门或看门狗代劳")
    if w.final_state is not None and w.final_state.stop_kind == w.final_state.STOPKIND_REQUEST \
            and len(w.finish_events) == 0:
        fails.append("无 finish 事件却出现 REQUEST 锁存停：锁存停的来源不是终点事件")
    if not fails:
        fails.append(f"负样本未产生任何失败征象（x_max={w.x_max:.3f} 未越过 runout_end_m={runout_end:.3f}）"
                     "：检查参考路径是否够长、墙钟预算是否足够让车冲过缓冲区")

if fails:
    print("ACCEL-FINISH SMOKE FAILED:")
    for f in fails:
        print("  -", f)
    sys.exit(1)
print("数据面断言通过")
PY
rc=$?
if [ "$rc" -ne 0 ]; then
    echo "FAIL: 数据面断言未通过（rc=$rc）；节点日志尾部："
    tail -n 3 "$OUT_DIR"/*.log
    exit 1
fi

# ── 4) 判据核单一来源守护：脚本侧 cap 必须与 #80 头文件算出的 cap 一致 ────────────
if [ "$ENABLE_FINISH_EVENT" = "1" ]; then
    node_cap="$(grep -oE 'cap=[0-9.]+' "$OUT_DIR/finish.log" | head -1 | tr -d 'cap=')"
    if [ -z "$node_cap" ]; then
        echo "FAIL: finish_event_node 未报告 cap（判据核未被真正调用？）"
        tail -n 5 "$OUT_DIR/finish.log"
        exit 1
    fi
    drift="$(python3 -c "import sys; print(1 if abs(float('$node_cap')-float('$CAP'))>0.01 else 0)")"
    if [ "$drift" = "1" ]; then
        echo "FAIL: cap 在两处漂移——脚本按 #80 公式算得 $CAP，节点报 $node_cap（差 >0.01）"
        exit 1
    fi
    echo "OK   cap 单一来源一致：脚本 $CAP == 节点 $node_cap（|Δ|<=0.01）"
fi

echo "Acceleration finish closed-loop smoke: PASS ✔  （ENABLE_FINISH_EVENT=$ENABLE_FINISH_EVENT）"
