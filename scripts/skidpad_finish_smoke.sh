#!/usr/bin/env bash
# ==============================================================================
# #91：八字绕环（skidpad）**ROS 闭环**完赛门禁（多节点 + 完赛事件 + 锁存停）。
#
# 与 acceleration_finish_smoke.sh（#82）的分工：那个证明直线赛"过线并停进缓冲区"；
# 本脚本证明八字赛"绕满双圆并经出场通道驶离"——同一仲裁器链拓扑、同一判据单一
# 来源约定，判据核换成 #90 的 skidpad_finish_judge.hpp（绕角累积计圈）。
#
# 拓扑（仲裁器链，单一权威出口）：
#   pathsource(内联发布 /planning/raw_pathlimits，#90 导出的八字中心线 349 点)
#     → velocity_profiler(→ /planning/pathlimits，曲率限速 + max_v 上限)
#     → pure_pursuit(→ /control/vehicle_command，源 A)
#     → command_arbiter_node(→ /vehicle_command，唯一出口；消费 /system/task_event)
#     → vehicle_simulator(物理积分 → /localization/vehicle_state 回流；init_x=-15 置于
#       参考线起点，赛道几何=skidpad_track.csv)
#   safety_monitor → /system/stop（可恢复 TIMEOUT 停）→ 仲裁器
#   finish_event_node：mode=skidpad，订阅 vehicle_state，首次完赛**恰好一次**发 finish
#     （#90 判据核：进场→右圆2圈→换圆→左圆2圈→出场）
#
# 数据面设计（与 accel 门禁的两处刻意差异，均有实测依据）：
#   1) 路径**只发一条**（等 profiler 订阅就绪后发布一次，之后停发）：PP 每条路径消息
#      都重置最近点索引并全局重锚，而八字中心线的进场/切点/出场段几何重叠在 x 轴上，
#      持续重发会被出场段劫持（本机实测：10Hz 重发 → 车直行冲出赛道）。单次发布下
#      PP 首次锚定唯一正确（车在唯一起点 -15），此后索引单调推进。八字赛程参考线
#      本就是静态已知的——这与真实系统"一次下发赛程参考线"的语义一致。
#   2) 参数覆盖三处（一次调整，issue #91 允许并要求记录；均为静态参考线场景的
#      时序适配，不改变任何安全语义的判停行为本身）：
#      - pure_pursuit safety.path_timeout 0.5→600 与 safety_monitor planner_timeout
#        0.5→600：单次发布后两侧输入看门狗不再把静态参考线判陈旧（vehicle_state 流
#        仍受 state_timeout 保护；TIMEOUT 停车链路本身不动——负样本仍由它兜底）；
#      - pure_pursuit algorithm.throttle.speed_high_threshold 2.0→10.0：默认值下
#        v≥2.3 后油门恒流 15%（为 accel 直线冲刺标定），速度环开环、车加速到
#        ~20 m/s；调到 10 让巡航段 PI 生效，实测稳定巡航 5.16 m/s、全程最大横向
#        偏差 0.434 m。
#
# 断言（正样本全部满足才算 PASS）：
#   - 预算内 finish 恰好一次（#90 状态机到 exit 相位）且 stop_kind==REQUEST（事件让
#     车停，不是看门狗 TIMEOUT——与 accel 门禁同一分界）；
#   - 横向走廊：全程对参考中心线最大横向偏差 ≤ 1.0 m（锥桶环 7.625/10.625 m、走廊
#     半宽 1.5 m，留 0.5 m 裕量；实测值写入 JSON 留痕）；
#   - 出场停稳：末速 < 0.1 m/s（沿 D-014 停稳语义）。
#
# 可证伪（#91 验收第 3 条）：
#   - ENABLE_FINISH_EVENT=0：不拉起终点事件源 → 无 finish、无 REQUEST 锁存，车绕完
#     八字停在路径末端（PP 末端制动），门禁必须失败且原因指向"未完赛/未停车"；
#   - REQUIRED_LAPS=N（>2）：判据核按 N 圈计数而车只按赛程绕 2 圈 → 永不 finish，
#     同样必须失败——这证明计圈真的在工作，不是"到点就 finish"。
#
# 判据阈值一律来自赛道元数据（#90 的 tracks.json skidpad 字段），不在本脚本硬编码
# 9.125 / 2。
#
# 本机注意：与既有 runtime 门禁一样需要可写 TMPDIR 与 FASTDDS_BUILTIN_TRANSPORTS=SHM
# （UDP 组播被 VPN 挡时 DDS 发现不可靠）；ros2 daemon 必须显式 start（见 PREWARM）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"
: "${TMPDIR:=$PWD/build/tmp}"
export TMPDIR
mkdir -p "$TMPDIR"

# PREWARM：先停残留 daemon，再**显式启动**并等待上图（#82 实测：仅 stop 指望后续
# 调用自动重建，在"每次调用都新建进程/会话"的执行器沙箱里不可靠，node list 恒空）。
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true
timeout 40 ros2 daemon start >/dev/null 2>&1 || true
sleep 3

OUT_DIR="${1:-build/skidpad_finish_smoke}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json

# ── 可调项（判据阈值来自元数据，这里只调预算与开关）────────────────────────────
BUDGET_SEC="${BUDGET_SEC:-180}"           # 数据面采样墙钟上限（实测全程 ~63s + 停稳余量）
ENABLE_FINISH_EVENT="${ENABLE_FINISH_EVENT:-1}"   # 0 = 可证伪负样本：不拉起终点事件源
REQUIRED_LAPS="${REQUIRED_LAPS:-}"        # 空 = 用赛道元数据；数字 = 负样本（如 3）
MAX_V="${MAX_V:-5.0}"                     # profiler 速度上限（巡航空速）
TRACK_ID="${TRACK_ID:-skidpad-figure8}"
TRACKS_DIR="${TRACKS_DIR:-src/simulation/vehicle_simulator/tracks}"
LATERAL_MAX="${LATERAL_MAX:-1.0}"         # 横向走廊断言阈值（m）
STOP_EPS="${STOP_EPS:-0.1}"               # 出场停稳末速阈值（m/s）

TRACK_CSV="$TRACKS_DIR/skidpad_track.csv"
[ -f "$TRACK_CSV" ] || { echo "FAIL: 赛道几何缺失 $TRACK_CSV"; exit 1; }

# ── 1) 完赛元数据来自 tracks.json（#90），不硬编码 9.125 / 2 ─────────────────────
EXPORT_DIR="$OUT_DIR/tracks_export"
mkdir -p "$EXPORT_DIR"
ros2 run track_benchmark benchmark_runner --export-tracks "$EXPORT_DIR" >/dev/null 2>"$OUT_DIR/export.log"
[ -f "$EXPORT_DIR/tracks.json" ] || { echo "FAIL: 未能导出 tracks.json"; tail -5 "$OUT_DIR/export.log"; exit 1; }
[ -f "$EXPORT_DIR/skidpad_centerline.csv" ] || { echo "FAIL: 未能导出 skidpad_centerline.csv（#90 中心线导出缺失）"; exit 1; }

if ! META_OUT="$(python3 - "$EXPORT_DIR/tracks.json" "$TRACK_ID" <<'PY'
import json, sys
man, tid = sys.argv[1], sys.argv[2]
t = next((x for x in json.load(open(man))["tracks"] if x["id"] == tid), None)
if t is None:
    sys.stderr.write(f"track {tid} 不在 tracks.json\n"); sys.exit(1)
m = t.get("skidpad")
if m is None:
    sys.stderr.write(f"{tid} 无 skidpad 元数据（字段为 null），本门禁不适用\n"); sys.exit(1)
print(f"{m['right_center_x']} {m['right_center_y']} {m['left_center_x']} {m['left_center_y']} "
      f"{m['path_radius']} {m['required_laps']} {m['exit_x']}")
PY
)"; then
    echo "FAIL: 无法从 tracks.json 取 skidpad 元数据（见上方 python 报错）"
    exit 1
fi
read -r RCX RCY LCX LCY RADIUS LAPS EXIT_X <<<"$META_OUT"
if [ -z "${RCX:-}" ] || [ -z "${RADIUS:-}" ] || [ -z "${LAPS:-}" ]; then
    echo "FAIL: 元数据解析结果不完整：[$META_OUT]"
    exit 1
fi
# 负样本第二开关：REQUIRED_LAPS 覆盖元数据圈数（> 实际圈数 → 永不 finish）。
if [ -n "$REQUIRED_LAPS" ]; then
    LAPS="$REQUIRED_LAPS"
    echo "=== 可证伪覆盖：required_laps=$LAPS（车按赛程只绕 2 圈 → 判据必须永不满足） ==="
fi
echo "=== 元数据（#90）：右圆($RCX,$RCY) 左圆($LCX,$LCY) r=$RADIUS laps=$LAPS exit_x=$EXIT_X ==="

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

# ── 2) 拉起闭环 ──────────────────────────────────────────────────────────────
echo "=== 拉起仲裁器闭环（墙钟预算 ${BUDGET_SEC}s，max_velocity=${MAX_V} m/s，ENABLE_FINISH_EVENT=$ENABLE_FINISH_EVENT） ==="
# 车置于参考线起点（进场通道 -15m）：首次全局锚定唯一正确，之后的单调推进是
# 八字（x 轴段几何重叠）能被跟踪的前提——见头部「数据面设计」。
start "$OUT_DIR/sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42 -p track_file:="$TRACK_CSV" \
    -p init_x:=-15.0 -p init_y:=0.0 -p init_theta:=0.0
start "$OUT_DIR/safety.log" ros2 run safety_monitor safety_monitor --ros-args \
    -p use_sim_time:=true -p planner_timeout:=600.0
start "$OUT_DIR/profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true -p limits.max_velocity:="$MAX_V"
# 一次性参考线模式的两处参数覆盖（头部「数据面设计」第 2 条）。
start "$OUT_DIR/pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
    -p use_sim_time:=true -p topics.vehicle_command:=/control/vehicle_command \
    -p safety.path_timeout:=600.0 -p algorithm.throttle.speed_high_threshold:=10.0
start "$OUT_DIR/arbiter.log" ros2 run safety_monitor command_arbiter_node --ros-args \
    -p use_sim_time:=true -p task.autostart:=true -p arbitration.source_timeout_sec:=0.5

EXPECT_NODES="vehicle_simulator safety_monitor velocity_profiler pure_pursuit command_arbiter_node"
if [ "$ENABLE_FINISH_EVENT" = "1" ]; then
    start "$OUT_DIR/finish.log" ros2 run track_benchmark finish_event_node --ros-args \
        -p use_sim_time:=true -p finish.mode:=skidpad -p finish.enabled:=true \
        -p skidpad.right_center_x:="$RCX" -p skidpad.right_center_y:="$RCY" \
        -p skidpad.left_center_x:="$LCX" -p skidpad.left_center_y:="$LCY" \
        -p skidpad.path_radius:="$RADIUS" -p skidpad.required_laps:="$LAPS" \
        -p skidpad.exit_x:="$EXIT_X"
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

# ── 3) 数据面：内联 rclpy 单次发布参考线 + 观测并落盘判据 JSON ───────────────────
python3 - "$EXPORT_DIR/skidpad_centerline.csv" "$OUT_DIR" "$LAPS" "$MAX_V" "$BUDGET_SEC" \
    "$STOP_EPS" "$LATERAL_MAX" "$ENABLE_FINISH_EVENT" <<'PY'
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatCarstate, HuatPathLimits, HuatSystemState, HuatVehicleCmd
from geometry_msgs.msg import Point
from std_msgs.msg import String

csv_path = sys.argv[1]
out_dir = sys.argv[2]
required_laps = int(sys.argv[3])
speed_limit = float(sys.argv[4])
budget, stop_eps = float(sys.argv[5]), float(sys.argv[6])
lateral_max = float(sys.argv[7])
expect_finish = sys.argv[8] == "1"

# 参考中心线（#90 导出，与生成器几何逐点一致）：349 点，进场(-15,0)→右圆×2→切点→左圆×2→出场(15,0)。
pts = []
for line in open(csv_path):
    if line.startswith("#"):
        continue
    f = line.split(",")
    pts.append((float(f[0]), float(f[1])))
if len(pts) < 100:
    print(f"FAIL: 参考线点数异常（{len(pts)} < 100）")
    sys.exit(2)


class Watch(Node):
    """单次发布八字参考线，并观测完赛判据与横向走廊。"""

    def __init__(self):
        super().__init__("skidpad_finish_watch")
        self.pub = self.create_publisher(HuatPathLimits, "/planning/raw_pathlimits", 10)
        self.published = False
        self.create_timer(0.2, self.tick)

        self.state_n = 0
        self.raw_path_n = 0
        self.pos = None
        self.v_last = 0.0
        self.v_max = 0.0
        self.max_dev = 0.0
        self.max_dev_at = None
        self.finish_events = []      # [(x, y, t_wall)]
        self.final_state = None
        self.cmd_n = 0
        self.cmd_bad_frame = 0
        self.cmd_bad_sum = 0
        self.stopped_since = None    # 首次 v<eps 的墙钟时刻（停稳计时）

        self.create_subscription(HuatCarstate, "/localization/vehicle_state", self.on_state, 50)
        self.create_subscription(HuatVehicleCmd, "/vehicle_command", self.on_cmd, 50)
        self.create_subscription(HuatSystemState, "/system/state", self.on_sys, 50)
        self.create_subscription(String, "/system/task_event", self.on_event, 50)
        self.create_subscription(HuatPathLimits, "/planning/raw_pathlimits",
                                 lambda m: setattr(self, "raw_path_n", self.raw_path_n + 1), 10)

    def tick(self):
        # 单次发布：等 velocity_profiler 订阅就绪再发一条全量参考线，之后停发。
        # （持续重发会让 PP 全局重锚被 x 轴重叠段劫持，见脚本头部说明。）
        if self.published or self.pub.get_subscription_count() < 1:
            return
        m = HuatPathLimits()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.path = [Point(x=p[0], y=p[1], z=0.0) for p in pts]
        m.replan = False
        self.pub.publish(m)
        self.published = True
        self.get_logger().info(f"八字参考线已发布（{len(pts)} 点，单次）")

    def on_state(self, m):
        self.state_n += 1
        x, y, v = m.car_state.x, m.car_state.y, m.v
        self.pos, self.v_last = (x, y), v
        self.v_max = max(self.v_max, v)
        # 横向走廊：本拍到参考中心线的最近距离（O(349) 粗搜足够；门禁只取全程最大值）。
        dmin = min((x - px) ** 2 + (y - py) ** 2 for px, py in pts)
        dev = math.sqrt(dmin)
        if dev > self.max_dev:
            self.max_dev, self.max_dev_at = dev, (round(x, 2), round(y, 2))
        if v < stop_eps:
            if self.stopped_since is None:
                self.stopped_since = time.time()
        else:
            self.stopped_since = None

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
        if m.data.strip().lower() == "finish":
            self.finish_events.append((self.pos or (0.0, 0.0)) + (round(time.time(), 1),))


rclpy.init()
w = Watch()
t0 = time.time()
while time.time() - t0 < budget:
    rclpy.spin_once(w, timeout_sec=0.1)
    if expect_finish and w.final_state is not None and w.finish_events:
        # 正样本提前退出：完赛事件 + REQUEST 锁存 + 停稳。
        if (w.final_state.stop_kind == w.final_state.STOPKIND_REQUEST and w.v_last < stop_eps):
            break
    elif not expect_finish and w.stopped_since is not None and time.time() - w.stopped_since > 3.0:
        # 负样本提前退出：车已停稳 3s 而完赛事件从未到来（失败征象确定）。
        break

snap = {
    "schema": "fsac.skidpad_finish_smoke/v1",
    "enable_finish_event": expect_finish,
    "required_laps": required_laps,
    "speed_limit_mps": speed_limit,
    "state_msgs": w.state_n,
    "raw_path_msgs": w.raw_path_n,
    "v_max": round(w.v_max, 3),
    "v_final": round(w.v_last, 3),
    "max_lateral_dev": round(w.max_dev, 3),
    "max_lateral_dev_at": w.max_dev_at,
    "lateral_max_allowed": lateral_max,
    "finish_event_count": len(w.finish_events),
    "finish_at_pos": [(round(a, 2), round(b, 2)) for a, b, _ in w.finish_events],
    "final_pos": [round(w.pos[0], 2), round(w.pos[1], 2)] if w.pos else None,
    "cmd_msgs": w.cmd_n,
    "cmd_bad_frame": w.cmd_bad_frame,
    "cmd_bad_checksum": w.cmd_bad_sum,
    "task_state": int(w.final_state.task_state) if w.final_state else -1,
    "stop_kind": int(w.final_state.stop_kind) if w.final_state else -1,
    "stop_active": bool(w.final_state.stop_active) if w.final_state else False,
    "can_drive": bool(w.final_state.can_drive) if w.final_state else True,
    "wall_used_s": round(time.time() - t0, 1),
    "budget_s": budget,
}
with open(f"{out_dir}/skidpad_finish_report.json", "w") as f:
    json.dump(snap, f, indent=2, sort_keys=True)
print("判据快照：")
for k, v in snap.items():
    print(f"  {k:<24}: {v}")
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
    # 正样本：完赛恰好一次 + REQUEST 锁存 + 横向走廊 + 出场停稳。
    if len(w.finish_events) != 1:
        fails.append(f"finish 事件应为恰好 1 次，实测 {len(w.finish_events)} 次"
                     "（0=完赛判据未触发——检查计圈/相位推进；>1=违反『恰好一次』）")
    if w.final_state is None:
        fails.append("未看到 /system/state（仲裁器遥测断链，锁存无法判定）")
    else:
        if w.final_state.stop_kind != w.final_state.STOPKIND_REQUEST:
            fails.append(f"停车来源不是 finish 的 REQUEST 锁存（stop_kind={int(w.final_state.stop_kind)}，"
                         "1=看门狗 TIMEOUT 而非完赛事件）")
        if not w.final_state.stop_active:
            fails.append("stop_active=false：锁存停未生效")
        if w.final_state.can_drive:
            fails.append("can_drive=true：锁存后仍被授予行驶许可")
    if w.max_dev > lateral_max:
        fails.append(f"横向走廊越界：max_lateral_dev={w.max_dev:.3f} m > {lateral_max} m（位置 {w.max_dev_at}）")
    if w.v_last >= stop_eps:
        fails.append(f"出场未停稳：末速度 v={w.v_last:.3f} >= eps={stop_eps} m/s")
else:
    # 负样本（ENABLE_FINISH_EVENT=0 或 REQUIRED_LAPS>2）：本门禁**必须非 0 退出**。
    # 每一条都是"失败原因"，不是"征象确认"——把征象当通过会让负样本 rc=0（#82 踩过的坑）。
    if len(w.finish_events) != 0:
        fails.append(f"终点事件源已禁用却仍收到 {len(w.finish_events)} 次 finish（禁用开关无效）")
    else:
        fails.append("未完赛：无 finish 事件 → 仲裁器未进入 REQUEST 锁存停"
                     f"（stop_kind={int(w.final_state.stop_kind) if w.final_state else -1}，"
                     f"末速 {w.v_last:.3f} m/s，final_pos {snap['final_pos']}——"
                     "车绕完/停在路径末端但完赛判定从未满足）")
    if w.final_state is not None and w.final_state.stop_kind == w.final_state.STOPKIND_REQUEST:
        fails.append("无 finish 事件却出现 REQUEST 锁存停：锁存停的来源不是完赛事件（判据存在后门）")

if fails:
    print("SKIDPAD-FINISH SMOKE FAILED:")
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

echo "Skidpad finish closed-loop smoke: PASS ✔  （ENABLE_FINISH_EVENT=$ENABLE_FINISH_EVENT REQUIRED_LAPS=$LAPS）"
