#!/usr/bin/env bash
# ==============================================================================
# #14：运行时接口契约检查 —— 关键话题的实际发布/订阅端点 + QoS 兼容性。
#
# 本脚本已升级为“闭环集成冒烟”：无界面拉起构成一条完整闭环链路的 4 个纯 C++ 节点
#   vehicle_simulator →(/localization/vehicle_state)→ velocity_profiler
#     →(/planning/pathlimits, 携带 target_speeds)→ pure_pursuit
#     →(/vehicle_command)→ vehicle_simulator   （回路闭合）
#   safety_monitor →(/system/stop, 锁存)→ pure_pursuit / vehicle_simulator
# 再用 `ros2 topic info -v` 断言：闭环上每条契约话题都有真实的 pub 与 sub 端点，
# 且 QoS 符合契约（stop=TRANSIENT_LOCAL 锁存 #8/#14；state/path/command=RELIABLE）。
# 这是 #17 闭环集成冒烟的起点：先验证契约/接线闭合，再在 #17 做带容差的时序集成。
#
# 仅用 `ros2 topic info`（确定性强、无数据流依赖），刻意不使用 `ros2 topic echo`
# （lyrical 上 echo teardown 有崩溃 bug）。所有 CLI 调用套 timeout，退出清理按 PID 精确执行。
#
# 退出清理注意（Jazzy CI 曾整步挂死 14 分钟，2026-09-22）：Jazzy 的 `ros2 run`
# wrapper (ros2run) 对 SIGINT 只吞掉 KeyboardInterrupt、既不转发也不退出，且它
# 启动的节点不在同一前台进程组、收不到 INT。因此清理必须对 wrapper 与节点 PID
# 各自递进升级 INT→TERM→KILL，并禁止无条件 `wait`（会永久阻塞）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1

PIDS=()       # ros2 run wrapper（ros2cli/python）进程
NODE_PIDS=()  # wrapper 的子进程，即真实节点可执行文件

# 对 wrapper + 节点递进升级信号直至 KILL；绝不无限等待。
cleanup() {
    local p c alive
    # wrapper 仍存活时补抓其节点子进程（覆盖未走到常规抓取点的提前退出路径）
    for p in "${PIDS[@]}"; do
        while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
    done
    for p in "${PIDS[@]}" "${NODE_PIDS[@]}"; do kill -INT "$p" 2>/dev/null || true; done
    for _ in $(seq 1 6); do
        alive=0
        for p in "${PIDS[@]}"; do kill -0 "$p" 2>/dev/null && alive=1; done
        [ "$alive" -eq 0 ] && break
        sleep 0.5
    done
    for p in "${PIDS[@]}" "${NODE_PIDS[@]}"; do kill -TERM "$p" 2>/dev/null || true; done
    sleep 0.5
    for p in "${PIDS[@]}" "${NODE_PIDS[@]}"; do kill -KILL "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true  # 子进程已被 KILL，wait 只回收，不会阻塞
}
trap cleanup EXIT

# 契约常量（与 common_msgs/interface_contract.h 对齐）；PP 指令话题显式对齐 kTopicVehicleCommand。
TOPIC_STATE="/localization/vehicle_state"
TOPIC_RAW_PATH="/planning/raw_pathlimits"
TOPIC_PATH="/planning/pathlimits"
TOPIC_COMMAND="/vehicle_command"
TOPIC_STOP="/system/stop"

ros2 run vehicle_simulator vehicle_simulator_node --ros-args -p use_sim_time:=true -p publish_clock:=true >build/qcc_sim.log 2>&1 & PIDS+=($!)
ros2 run safety_monitor safety_monitor >build/qcc_safety.log 2>&1 & PIDS+=($!)
ros2 run velocity_profiler velocity_profiler_node >build/qcc_profiler.log 2>&1 & PIDS+=($!)
# PP 默认指令话题为 /control/vehicle_command；此处按契约常量覆盖为 /vehicle_command，
# 使 sim→profiler→pp→sim 闭环在契约话题上真正闭合（演示 #14 项① 话题名统一）。
ros2 run pure_pursuit pure_pursuit_controller --ros-args -p topics.vehicle_command:="$TOPIC_COMMAND" \
    >build/qcc_pp.log 2>&1 & PIDS+=($!)

# 等四节点上节点图（最多 ~25s；单次查询限时，避免 CLI/daemon 异常时无限阻塞）
ready=0
for _ in $(seq 1 25); do
    nodes="$(timeout 8 ros2 node list 2>/dev/null)"
    if grep -q vehicle_simulator <<<"$nodes" && grep -q safety_monitor <<<"$nodes" &&
        grep -q velocity_profiler <<<"$nodes" && grep -q pure_pursuit <<<"$nodes"; then
        ready=1
        break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: nodes did not register; sim/safety/profiler/pp logs:"
    tail -5 build/qcc_*.log
    exit 1
fi
# 记录真实节点进程 PID（wrapper 的直接子进程）：退出清理按 PID 精确执行，
# 既不依赖 wrapper 转发信号（Jazzy 不转发），也不按进程名匹配（comm 名 15 字符截断）
for p in "${PIDS[@]}"; do
    while read -r c; do NODE_PIDS+=("$c"); done < <(pgrep -P "$p" 2>/dev/null || true)
done
# 给发现层一点时间收敛端点
sleep 3

fail=0

# count_field <topic> <regex> -> 输出匹配数（单次查询限时）
info_of() { timeout 10 ros2 topic info "$1" -v 2>/dev/null; }

# check_endpoints <topic> [need_pub=1] [need_sub=1]
# 闭环上除外部输入 raw_pathlimits（其发布者是规划器，本冒烟不拉起）外，均需 pub 与 sub 各 >=1。
check_endpoints() {
    local t="$1"
    local need_pub="${2:-1}"
    local need_sub="${3:-1}"
    local out pub sub
    out="$(info_of "$t")"
    pub="$(grep -oE 'Publisher count: [0-9]+' <<<"$out" | grep -oE '[0-9]+')"
    sub="$(grep -oE 'Subscription count: [0-9]+' <<<"$out" | grep -oE '[0-9]+')"
    pub="${pub:-0}"
    sub="${sub:-0}"
    local pub_ok=1 sub_ok=1
    [ "$need_pub" -eq 1 ] && [ "$pub" -lt 1 ] && pub_ok=0
    [ "$need_sub" -eq 1 ] && [ "$sub" -lt 1 ] && sub_ok=0
    if [ "$pub_ok" -eq 1 ] && [ "$sub_ok" -eq 1 ]; then
        echo "OK   $t: pub=$pub sub=$sub"
        return 0
    else
        echo "FAIL $t: pub=$pub sub=$sub (need pub>=$need_pub sub>=$need_sub)"
        return 1
    fi
}

echo "=== 闭环端点存在性（sim→profiler→pp→sim + safety） ==="
check_endpoints "$TOPIC_STATE" 1 1 || fail=1       # pub: sim;  sub: profiler/pp
check_endpoints "$TOPIC_RAW_PATH" 0 1 || fail=1    # sub: profiler（发布者是外部规划器，不断言 pub）
check_endpoints "$TOPIC_PATH" 1 1 || fail=1        # pub: profiler;  sub: pp
check_endpoints "$TOPIC_COMMAND" 1 1 || fail=1     # pub: pp;  sub: sim
check_endpoints "$TOPIC_STOP" 1 1 || fail=1        # pub: safety;  sub: pp/sim

echo "=== QoS 兼容（stop 锁存 / state·path·command 可靠） ==="
stop_out="$(info_of "$TOPIC_STOP")"
# 至少一个 TRANSIENT_LOCAL 端点（发布端），保证晚加入者收到最后停车态（#8）
if grep -q "Durability: TRANSIENT_LOCAL" <<<"$stop_out"; then
    echo "OK   $TOPIC_STOP 有 TRANSIENT_LOCAL 端点（锁存契约）"
else
    echo "FAIL $TOPIC_STOP 缺少 TRANSIENT_LOCAL 端点"
    printf '%s\n' "$stop_out" | sed 's/^/     /'
    fail=1
fi
for t in "$TOPIC_STATE" "$TOPIC_PATH" "$TOPIC_COMMAND"; do
    out="$(info_of "$t")"
    if grep -q "Reliability: RELIABLE" <<<"$out"; then
        echo "OK   $t RELIABLE"
    else
        echo "FAIL $t 非 RELIABLE"
        fail=1
    fi
done

if [ "$fail" -ne 0 ]; then
    echo "Closed-loop contract/QoS integration smoke: FAILED"
    exit 1
fi
echo "Closed-loop contract/QoS integration smoke: PASS ✔"
