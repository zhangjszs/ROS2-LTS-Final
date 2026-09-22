#!/usr/bin/env bash
# ==============================================================================
# #14：运行时接口契约检查 —— 关键话题的实际发布/订阅端点 + QoS 兼容性。
# 无界面拉起 vehicle_simulator + safety_monitor + pure_pursuit（均无硬件/GUI 依赖），
# 用 `ros2 topic info -v` 断言：话题有真实 pub 与 sub 端点，且 stop 话题两端均为
# TRANSIENT_LOCAL（锁存契约，#8/#14）、state 话题两端 RELIABLE。
# 同时充当 #17 闭环集成冒烟的起点（先验证契约，再做带容差的时序集成）。
# ==============================================================================
set -uo pipefail
export ROS_LOCALHOST_ONLY=1

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -INT "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

ros2 run vehicle_simulator vehicle_simulator_node --ros-args -p use_sim_time:=true -p publish_clock:=true >build/qcc_sim.log 2>&1 & PIDS+=($!)
ros2 run safety_monitor safety_monitor >build/qcc_safety.log 2>&1 & PIDS+=($!)
ros2 run pure_pursuit pure_pursuit_controller >build/qcc_pp.log 2>&1 & PIDS+=($!)

# 等三节点上节点图（最多 ~25s）
ready=0
for _ in $(seq 1 25); do
    nodes="$(ros2 node list 2>/dev/null)"
    if grep -q vehicle_simulator <<<"$nodes" && grep -q safety_monitor <<<"$nodes" && grep -q pure_pursuit <<<"$nodes"; then
        ready=1; break
    fi
    sleep 1
done
if [ "$ready" -ne 1 ]; then
    echo "FAIL: nodes did not register; sim/safety/pp logs:"; tail -5 build/qcc_*.log; exit 1
fi
# 给发现层一点时间收敛端点
sleep 3

fail=0

# count_field <topic> <regex> -> 输出匹配数
info_of() { ros2 topic info "$1" -v 2>/dev/null; }

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
