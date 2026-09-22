#!/usr/bin/env bash
# ==============================================================================
# #14：运行时接口契约检查 —— 关键话题的实际发布/订阅端点 + QoS 兼容性。
# 无界面拉起 vehicle_simulator + safety_monitor + pure_pursuit（均无硬件/GUI 依赖），
# 用 `ros2 topic info -v` 断言：话题有真实 pub 与 sub 端点，且 stop 话题两端均为
# TRANSIENT_LOCAL（锁存契约，#8/#14）、state 话题两端 RELIABLE。
# 同时充当 #17 闭环集成冒烟的起点（先验证契约，再做带容差的时序集成）。
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

ros2 run vehicle_simulator vehicle_simulator_node --ros-args -p use_sim_time:=true -p publish_clock:=true >build/qcc_sim.log 2>&1 & PIDS+=($!)
ros2 run safety_monitor safety_monitor >build/qcc_safety.log 2>&1 & PIDS+=($!)
ros2 run pure_pursuit pure_pursuit_controller >build/qcc_pp.log 2>&1 & PIDS+=($!)

# 等三节点上节点图（最多 ~25s；单次查询限时，避免 CLI/daemon 异常时无限阻塞）
ready=0
for _ in $(seq 1 25); do
    nodes="$(timeout 8 ros2 node list 2>/dev/null)"
    if grep -q vehicle_simulator <<<"$nodes" && grep -q safety_monitor <<<"$nodes" && grep -q pure_pursuit <<<"$nodes"; then
        ready=1; break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: nodes did not register; sim/safety/pp logs:"; tail -5 build/qcc_*.log; exit 1
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

check_endpoints() {  # topic
    local t="$1"; local out; out="$(info_of "$t")"
    local pub sub
    pub="$(grep -oE 'Publisher count: [0-9]+' <<<"$out" | grep -oE '[0-9]+')"
    sub="$(grep -oE 'Subscription count: [0-9]+' <<<"$out" | grep -oE '[0-9]+')"
    pub="${pub:-0}"; sub="${sub:-0}"
    if [ "$pub" -ge 1 ] && [ "$sub" -ge 1 ]; then
        echo "OK   $t: pub=$pub sub=$sub"; return 0
    else
        echo "FAIL $t: pub=$pub sub=$sub (need >=1 each)"; return 1
    fi
}

echo "=== 端点存在性 ==="
check_endpoints /localization/vehicle_state || fail=1
check_endpoints /system/stop || fail=1
# /vehicle_command：PP 仅在收到路径后才创建发布者（无输入不发指令），故仅信息性报告，不作硬断言。
echo "info /vehicle_command: $(info_of /vehicle_command | grep -oE '(Publisher|Subscription) count: [0-9]+' | tr '\n' ' ')"

echo "=== QoS 兼容（stop 锁存 / state 可靠） ==="
stop_out="$(info_of /system/stop)"
# 至少一个 TRANSIENT_LOCAL 端点（发布端），且无 RELIABLE 不匹配（两端均 RELIABLE）
if grep -q "Durability: TRANSIENT_LOCAL" <<<"$stop_out"; then
    echo "OK   /system/stop 有 TRANSIENT_LOCAL 端点（锁存契约）"
else
    echo "FAIL /system/stop 缺少 TRANSIENT_LOCAL 端点"; printf '%s\n' "$stop_out" | sed 's/^/     /'; fail=1
fi
st_out="$(info_of /localization/vehicle_state)"
if grep -q "Reliability: RELIABLE" <<<"$st_out"; then
    echo "OK   /localization/vehicle_state RELIABLE"
else
    echo "FAIL /localization/vehicle_state 非 RELIABLE"; fail=1
fi

if [ "$fail" -ne 0 ]; then echo "QoS/contract runtime check: FAILED"; exit 1; fi
echo "QoS/contract runtime check: PASS ✔"
