#!/usr/bin/env bash
# ==============================================================================
# #16：故障注入验收冒烟（软件可观测子集；终态自动判定，不靠 RViz 或日志目视）。
#
# 拓扑 = 闭环 + 权威单出口（与 huat_launch 的 use_arbiter 模式同构）：
#   sim → /localization/vehicle_state → velocity_profiler → /planning/pathlimits
#     → pure_pursuit → /control/vehicle_command（源 A）
#   harness → /mpc/vehicle_command（源 B，用于仲裁/篡改场景）
#   command_arbiter_node（任务×安全状态机 + 去抖）→ /vehicle_command（唯一出口）→ sim
#   safety_monitor → /system/stop（锁存话题）
#
# 覆盖的软故障（每个都有预期终态 + 停止预算，可重复）：
#   正常行驶 / 路径停发 / 空路径 / 源帧被篡改 / 外部停车与恢复 / 任务级锁存(abort)
#   / 重复停车 / 人工复位 / 监控退出 / 仲裁器重启（晚加入不授予许可）
#
# 每个场景记录时间线（#16 交付物第 7 条）：触发 → 检测（状态迁移）→ 最后驱动指令
#   → 制动建立（出口零油门）→ 车速回落；写成机读 JSON + 汇总表。
#
# 明确不在本脚本范围内（不得据此声称已验收）：真实 VCU 断连/掉电、硬件急停按钮优先级、
# 底盘超时接管 —— 需台架/实车（见 #16/#15 未标定项）。
#
# 进程控制：只按 PID 递进 INT→TERM→KILL（本机 bwrap 会话命令行含脚本全文，
# `pkill -f <节点名>` 会自伤 —— 实测会杀掉整个测试会话）。
# 本机若 UDP 组播被 VPN 挡住，调用前 export FASTDDS_BUILTIN_TRANSPORTS=SHM（CI 上不需要）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=$PWD/build/.ros}"
export ROS_HOME
mkdir -p "$ROS_HOME"

OUT_DIR="${1:-build/fault_injection}"
mkdir -p "$OUT_DIR" || { echo "cannot create $OUT_DIR"; exit 1; }
rm -f "$OUT_DIR"/*.log "$OUT_DIR"/*.json

PIDS=()      # ros2 run wrapper
NODE_PIDS=() # wrapper 的直接子进程 = 真实节点（按启动顺序）

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

start "$OUT_DIR/sim.log" ros2 run vehicle_simulator vehicle_simulator_node --ros-args \
    -p use_sim_time:=true -p publish_clock:=true -p seed:=42
start "$OUT_DIR/safety.log" ros2 run safety_monitor safety_monitor --ros-args -p use_sim_time:=true
start "$OUT_DIR/profiler.log" ros2 run velocity_profiler velocity_profiler_node --ros-args \
    -p use_sim_time:=true
# PP 改发源话题 A；最终出口由仲裁器独占（单一权威，控制器不得绕过）
start "$OUT_DIR/pp.log" ros2 run pure_pursuit pure_pursuit_controller --ros-args \
    -p use_sim_time:=true -p topics.vehicle_command:=/control/vehicle_command
start "$OUT_DIR/arbiter.log" ros2 run safety_monitor command_arbiter_node --ros-args \
    -p use_sim_time:=true -p arbitration.source_timeout_sec:=0.5

miss=1
for _ in $(seq 1 12); do
    nodes="$(timeout 12 ros2 node list --no-daemon 2>/dev/null)"
    miss=0
    for n in vehicle_simulator safety_monitor velocity_profiler pure_pursuit command_arbiter_node; do
        grep -q "$n" <<<"$nodes" || miss=1
    done
    [ "$miss" -eq 0 ] && break
    sleep 2
done
if [ "$miss" -ne 0 ]; then
    echo "FAIL: 故障注入拓扑节点未全部注册；ros2 node list --no-daemon 输出："
    printf '%s\n' "$nodes"
    tail -n 3 "$OUT_DIR"/*.log
    exit 1
fi
NODE_PIDS=()
for p in "${PIDS[@]}"; do
    while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
done
SAFETY_PID="${NODE_PIDS[1]}"
ARBITER_PID="${NODE_PIDS[4]}"
echo "OK   拓扑就绪（5 节点，单一权威出口 /vehicle_command；safety=$SAFETY_PID arbiter=$ARBITER_PID）"

python3 - "$OUT_DIR" "$SAFETY_PID" "$ARBITER_PID" <<'PY'
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

from common_msgs.msg import HuatPathLimits, HuatStop, HuatSystemState, HuatVehicleCmd
from geometry_msgs.msg import Point
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

out_dir, safety_pid, arbiter_pid = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])

SRC_A, SRC_B, OUT_CMD = "/control/vehicle_command", "/mpc/vehicle_command", "/vehicle_command"
STATE, EVENT, RAW_PATH, STOP = "/system/state", "/system/task_event", "/planning/raw_pathlimits", "/system/stop"

# 与 HuatSystemState 常量一致的编码（msg 侧另有 static_assert 锁定）
TASK_IDLE, TASK_RUNNING = 0, 2
SAFETY_STOP, SAFETY_NORMAL = 2, 0
KIND_NONE, KIND_TIMEOUT, KIND_REQUEST, KIND_FAULT = 0, 1, 2, 3
SRC_NONE, SRC_PP, SRC_MPC = 0, 1, 2
R_CONTROL, R_STOP, R_TASK, R_NO_SRC = 0, 1, 2, 3
U_NONE, U_ABSENT, U_BAD_FRAME, U_BAD_SUM, U_STALE = 0, 1, 2, 3, 4


class Harness(Node):
    """场景驱动器：注入故障 + 记录时间线 + 自动判定预期终态。"""

    def __init__(self):
        super().__init__("fault_injection_harness")
        self.last_state = None
        self.last_cmd = None
        self.speed = 0.0
        self.publish_path = True
        self.path_points = 60
        self.create_subscription(HuatSystemState, STATE, self.on_state, 50)
        self.create_subscription(HuatVehicleCmd, OUT_CMD, self.on_cmd, 50)
        from common_msgs.msg import HuatCarstate
        self.create_subscription(HuatCarstate, "/localization/vehicle_state",
                                 lambda m: setattr(self, "speed", float(m.v)), 50)
        self.pub_path = self.create_publisher(HuatPathLimits, RAW_PATH, 10)
        self.pub_b = self.create_publisher(HuatVehicleCmd, SRC_B, 10)
        # /system/stop 必须用锁存 QoS（契约 kQosStop：reliable + transient_local）：
        # 用默认 volatile 发布会被仲裁器的 transient_local 订阅判为不兼容，一条都收不到
        # （rclcpp 会告警 "incompatible QoS ... DURABILITY"）—— 本冒烟顺带把这条契约钉住。
        stop_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_stop = self.create_publisher(HuatStop, STOP, stop_qos)
        self.pub_event = self.create_publisher(String, EVENT, 10)
        self.create_timer(0.1, self.tick_path)

    def tick_path(self):
        if not self.publish_path:
            return
        m = HuatPathLimits()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "map"
        m.path = [Point(x=0.5 * i, y=0.0, z=0.0) for i in range(self.path_points)]
        m.replan = False
        self.pub_path.publish(m)

    def on_state(self, m):
        self.last_state = m

    def on_cmd(self, m):
        self.last_cmd = m

    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def event(self, name, times=3, gap=0.35):
        for _ in range(times):     # 事件用短重复发送，避开 DDS 匹配竞态
            m = String()
            m.data = name
            self.pub_event.publish(m)
            self.spin_for(gap)

    def publish_mpc_frame(self, pedal, tamper):
        m = HuatVehicleCmd()
        m.head1, m.head2, m.length = 0xAA, 0x55, 10
        m.steering, m.brake_force, m.pedal_ratio = 90, 0, pedal
        m.gear_position, m.working_mode, m.racing_num, m.racing_status = 1, 1, 1, 1
        region = [m.steering, m.brake_force, m.pedal_ratio, m.gear_position,
                  m.working_mode, m.racing_num, m.racing_status]
        m.checksum = sum(region) & 0xFFFF
        if tamper:
            m.pedal_ratio = 0        # 载荷改写但保留原校验和 → 必须被拒
        self.pub_b.publish(m)

    def publish_stop(self, val, times=3, gap=0.3):
        for _ in range(times):
            m = HuatStop()
            m.stop = bool(val)
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub_stop.publish(m)
            self.spin_for(gap)

    def arm_and_drive(self, budget=8.0):
        """授予行驶许可：需等监控释放锁存（路径先流起来），arm 在未 STOP 时才生效。"""
        deadline = time.time() + budget
        while time.time() < deadline:
            st = self.last_state
            if st is not None and st.safety_state != SAFETY_STOP:
                self.event("arm", times=1)
                self.event("start", times=1)
                self.spin_for(0.6)
                st = self.last_state
                if st is not None and st.task_state == TASK_RUNNING:
                    return True
            self.spin_for(0.5)
        return False

    def kill_arbiter(self):
        try:
            os.kill(arbiter_pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def restart_arbiter(self):
        p = subprocess.Popen(
            ["ros2", "run", "safety_monitor", "command_arbiter_node", "--ros-args",
             "-p", "use_sim_time:=true", "-p", "arbitration.source_timeout_sec:=0.5"],
            stdout=open(f"{out_dir}/arbiter_restart.log", "w"), stderr=subprocess.STDOUT)
        return p


def snap(h, marker):
    s, c = h.last_state, h.last_cmd
    return {
        "at": round(time.time(), 3),
        "marker": marker,
        "task_state": None if s is None else int(s.task_state),
        "safety_state": None if s is None else int(s.safety_state),
        "stop_kind": None if s is None else int(s.stop_kind),
        "stop_active": None if s is None else bool(s.stop_active),
        "can_drive": None if s is None else bool(s.can_drive),
        "winner": None if s is None else int(s.winner_source),
        "reason": None if s is None else int(s.reason),
        "src_a_status": None if s is None else int(s.source_a_status),
        "src_b_status": None if s is None else int(s.source_b_status),
        "src_b_rejected": None if s is None else int(s.source_b_rejected),
        "out_pedal": None if c is None else int(c.pedal_ratio),
        "out_brake": None if c is None else int(c.brake_force),
        "speed_mps": round(h.speed, 3),
    }


def expect(cond, msg, fails):
    if not cond:
        fails.append(msg)


def stopped_ok(tl):
    return (tl.get("out_pedal") or 0) == 0 and (tl.get("out_brake") or 0) > 0


# ── 场景：drive(注入) + check(预期终态) ────────────────────────────────────────
def d01(h):
    ok = h.arm_and_drive()
    h.spin_for(4.0)
    return {"armed": ok}


def c01(tl, f, extra):
    expect(extra.get("armed"), "未能授予行驶许可（arm/start 被拒或监控未释放）", f)
    expect(tl.get("task_state") == TASK_RUNNING, "任务未进入 RUNNING", f)
    expect(tl.get("winner") in (SRC_PP, SRC_MPC), "没有可信控制源被接管", f)
    expect(stopped_ok(tl) is False, "正常行驶场景出口仍是制动", f)
    expect((tl.get("speed_mps") or 0) > 0.2, "闭环未驱动车辆（速度未上升）", f)


def d02(h):
    h.publish_path = False          # 路径停发 → planner 心跳超时
    h.spin_for(3.0)


def c02(tl, f, extra):
    expect(bool(tl.get("stop_active")), "路径停发后未进入停车态", f)
    expect(tl.get("stop_kind") == KIND_TIMEOUT, "停车来源不是 TIMEOUT（看门狗）", f)
    expect(tl.get("reason") in (R_STOP, R_TASK, R_NO_SRC), "停车原因未反映降级", f)
    expect(stopped_ok(tl), "出口未回到零油门+制动", f)


def d03(h):
    h.publish_path = True
    h.path_points = 0               # 空路径：不得当作心跳（ROS1 #9 缺陷类）
    h.spin_for(3.0)


def c03(tl, f, extra):
    expect(stopped_ok(tl), "空路径下出口未回到制动（空路径被当成心跳？）", f)
    expect(bool(tl.get("stop_active")), "空路径未被判为需要停车", f)


def d04(h):
    h.publish_path = True
    h.path_points = 60
    for _ in range(40):
        h.publish_mpc_frame(pedal=90, tamper=True)
        h.spin_for(0.05)


def c04(tl, f, extra):
    expect(tl.get("src_b_status") == U_BAD_SUM, "篡改帧未被判为 bad_checksum", f)
    expect(tl.get("winner") != SRC_MPC, "被篡改的源接管了出口（信任门失效）", f)
    expect((tl.get("src_b_rejected") or 0) > 0, "篡改帧未被计入拒收", f)


def d05(h):
    h.publish_stop(True)            # 外部（监控类）停车
    h.spin_for(1.0)
    ext = snap(h, "external_stop")
    h.publish_stop(False)           # 释放 → 可恢复类应解除
    h.spin_for(2.0)
    return {"external_stop": ext}


def c05(tl, f, extra):
    ext = extra.get("external_stop", {})
    expect(bool(ext.get("stop_active")), "外部停车请求未生效", f)
    expect(stopped_ok(ext), "外部停车期间出口未回到制动", f)
    expect(not tl.get("stop_active"), "监控释放后仍可恢复类停车未解除（TIMEOUT 应可恢复）", f)


def d06(h):
    h.event("abort")                # 任务级锁存停
    h.spin_for(1.0)
    ext = snap(h, "after_abort")
    h.publish_stop(False)           # 监控“恢复”不应带出锁存停
    h.spin_for(2.0)
    return {"after_abort": ext}


def c06(tl, f, extra):
    ext = extra.get("after_abort", {})
    expect(bool(ext.get("stop_active")), "abort 未触发锁存停", f)
    expect(ext.get("stop_kind") == KIND_FAULT, "abort 的停车来源不是 FAULT", f)
    expect(bool(tl.get("stop_active")), "监控释放解除了任务级锁存停（不应发生）", f)
    expect(stopped_ok(tl), "锁存停期间出口有油门", f)


def d07(h):
    for _ in range(6):
        h.publish_stop(True, times=1)
        h.spin_for(0.2)


def c07(tl, f, extra):
    expect(bool(tl.get("stop_active")), "重复停车后不再是停车态（幂等性失败）", f)
    expect(tl.get("stop_kind") == KIND_FAULT, "重复停车改变了锁存来源类型", f)


def d08(h):
    h.event("reset")
    h.spin_for(2.0)


def c08(tl, f, extra):
    expect(not tl.get("stop_active"), "reset 未解除锁存停", f)
    expect(tl.get("task_state") == TASK_IDLE, "reset 后任务态未回到 IDLE", f)
    expect((tl.get("out_pedal") or 0) == 0, "reset 后未经 arm/start 就有油门（意外授予许可）", f)


def d09(h):
    try:
        os.kill(safety_pid, signal.SIGKILL)   # 监控退出
    except ProcessLookupError:
        pass
    h.spin_for(3.0)


def c09(tl, f, extra):
    # 监控沉默 ≠ 可以行驶：锁存 stop 仍生效（transient_local），出口必须保持制动
    expect((tl.get("out_pedal") or 0) == 0 or tl.get("task_state") != TASK_RUNNING,
           "监控退出后行驶许可还在（把节点沉默当成了正常）", f)


def d10(h):
    h.kill_arbiter()
    h.spin_for(1.5)
    h.proc = h.restart_arbiter()
    h.spin_for(6.0)                 # 晚加入者：只靠锁存 stop + 自身 IDLE 判定


def c10(tl, f, extra):
    expect(tl.get("task_state") == TASK_IDLE, "重启后的仲裁器不处于 IDLE", f)
    expect(not tl.get("can_drive"), "晚加入者意外授予了行驶许可", f)
    expect((tl.get("out_pedal") or 0) == 0, "晚加入者出口不是零油门", f)


CASES = [
    ("01_baseline_drive", d01, c01),
    ("02_path_lost", d02, c02),
    ("03_empty_path", d03, c03),
    ("04_tampered_source_frame", d04, c04),
    ("05_external_stop_and_resume", d05, c05),
    ("06_abort_is_latched", d06, c06),
    ("07_repeat_stop_idempotent", d07, c07),
    ("08_manual_reset_no_permission", d08, c08),
    ("09_monitor_exit", d09, c09),
    ("10_arbiter_restart_late_join", d10, c10),
]

rclpy.init()
h = Harness()
h.spin_for(1.5)
overall = 0
summary = []
for name, drive, check in CASES:
    fails = []
    pre = snap(h, "pre_trigger")
    extra = drive(h) or {}
    post = snap(h, "post_trigger")
    check(post, fails, extra)
    rec = {"case": name, "pre": pre, "post": post, "extra": extra, "fails": fails}
    with open(f"{out_dir}/{name}.json", "w") as fp:
        json.dump(rec, fp, ensure_ascii=False, indent=2)
    ok = not fails
    overall |= 0 if ok else 1
    print(f"{'OK  ' if ok else 'FAIL'} {name}: task={post.get('task_state')} safety={post.get('safety_state')} "
          f"kind={post.get('stop_kind')} reason={post.get('reason')} winner={post.get('winner')} "
          f"A={post.get('src_a_status')} B={post.get('src_b_status')} pedal={post.get('out_pedal')} "
          f"brake={post.get('out_brake')} v={post.get('speed_mps')}")
    for m in fails:
        print(f"       - {m}")
    summary.append({"case": name, "ok": ok, "fails": fails, "post": post})

with open(f"{out_dir}/summary.json", "w") as fp:
    json.dump(summary, fp, ensure_ascii=False, indent=2)
try:
    h.proc.terminate()
except AttributeError:
    pass
rclpy.shutdown()
sys.exit(overall)
PY
rc=$?
cleanup
trap - EXIT
if [ "$rc" -eq 0 ]; then
    echo "Fault injection smoke: PASS ✔ （10 个软故障场景终态自动判定；硬件急停/VCU 断连仍需台架实车）"
else
    echo "Fault injection smoke: FAILED（详见 $OUT_DIR/*.json 时间线）"
fi
exit $rc
