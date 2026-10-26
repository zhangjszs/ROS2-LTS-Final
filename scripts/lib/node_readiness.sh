#!/usr/bin/env bash
# ==============================================================================
# #69：多节点门禁的"就绪探测"共用件（纯 bash，被门禁脚本 source）。
#
# ── 为什么需要（Issue #69；根因证据：main 的 push run 40205834746 判红）─────
# 旧实现把"节点是否就绪"等价于"ros2 daemon 的 `ros2 node list` 视图能否看到
# 节点"。那次 CI 里门禁报"节点未全部注册 / ros2 node list 空表"，而**同一轮日志
# 中 6 个节点的进程日志全部正常**（sim.log 打印了 108 锥桶与初始位姿、
# profiler.log 打印 `Initialized. max_v=5.0 m/s`、pp.log 持续输出
# `No path received yet, braking`）——即节点全都在跑，只有 daemon 的发现视图是
# 空表。**这是就绪探测的单点失效**，不是环境没配好：再多 daemon 预热也修不好。
#
# ── 本件的契约 ──────────────────────────────────────────────────────────────
#   1) **以进程存活为准**：门禁脚本持有每个节点的启动 PID，进程退出即真失败。
#      注意：bash 后台任务退出后会先变成僵尸，`kill -0` 对僵尸仍然返回成功，
#      因此存活判定必须看进程状态（`ps -o stat=` 的 Z 态）而不是只看 kill -0。
#   2) daemon 的 `ros2 node list` **降级为辅助确认**：视图为空**不判红**。
#   3) 两类失败必须可区分：
#      - 真失败  = 目标进程已退出 → 立即判红，并明确指出**是哪个节点**没起来；
#      - 环境性  = 进程全活但 daemon 视图空 → 走完既有重试后按"疑环境性失败"
#                  提示继续，**最终判红与否由数据面断言决定**，不得伪装成
#                  "节点没起来"。
#   4) **不得弱化门禁**：真失败（节点真的没起来）必须仍然非 0 退出。
#
# ── 用法 ────────────────────────────────────────────────────────────────────
#   source "$(dirname "${BASH_SOURCE[0]}")/lib/node_readiness.sh"
#   start_node <节点名> <日志> <命令...>      # 启动并登记
#   probe_wait_ready <期望节点列表> <重试次数> <间隔秒>
#       返回 0 = daemon 视图确认就绪
#       返回 1 = 真失败（有节点进程已退出）——调用方必须判红
#       返回 2 = 环境性（进程全活但 daemon 视图空）——调用方继续并提示
# ==============================================================================

# 已登记节点：名称与启动 PID 一一对应（并行数组）。
PROBE_NODE_NAMES=()
PROBE_NODE_PIDS=()

# 节点视图查询命令（默认真实 daemon 查询；自测可注入假实现）。
# 之所以留这个钩子：#69 判据 1/3 要求"可构造'进程全活但视图空'与'某节点进程
# 立刻退出'两个情形并给出证据"，没有注入点就只能靠运气复现。
PROBE_NODE_LIST_CMD="${PROBE_NODE_LIST_CMD:-ros2 node list}"

# 探测结果（供调用方读取）：
#   PROBE_DEAD_NODES    —— 已退出的节点名（换行分隔）
#   PROBE_LAST_NODE_VIEW —— 最后一次 daemon 节点视图
#   PROBE_ENV_SUSPECT  —— 1 = 疑似环境性失败（进程活但视图空）
PROBE_DEAD_NODES=""
PROBE_LAST_NODE_VIEW=""
PROBE_ENV_SUSPECT=0

# 登记一个已启动的节点进程。
# 参数：$1 = 节点名（ROS 图上的名字）  $2 = 启动 PID
probe_register() {
    PROBE_NODE_NAMES+=("$1")
    PROBE_NODE_PIDS+=("$2")
}

# 进程是否存活（排除僵尸态）。
# 参数：$1 = PID；返回 0 = 活着，1 = 已死或僵尸。
probe_proc_alive() {
    local st
    if ! command -v ps >/dev/null 2>&1; then
        kill -0 "$1" 2>/dev/null
        return
    fi
    st="$(ps -o stat= -p "$1" 2>/dev/null | tr -d ' ')" || return 1
    [ -n "$st" ] || return 1
    [ "${st:0:1}" != "Z" ]
}

# 打印已经退出的节点名（每行一个）；全部存活则无输出。
probe_dead_nodes() {
    local i
    for i in "${!PROBE_NODE_NAMES[@]}"; do
        probe_proc_alive "${PROBE_NODE_PIDS[$i]}" || printf '%s\n' "${PROBE_NODE_NAMES[$i]}"
    done
}

# 就绪探测主循环。
# 参数：$1 = 期望节点名（空格分隔）  $2 = 重试次数  $3 = 每轮间隔秒
# 返回：0 = daemon 视图确认就绪 / 1 = 真失败 / 2 = 环境性（进程活、视图空）
probe_wait_ready() {
    local expect="$1" retries="$2" nap="$3"
    local nodes="" missing n dead i
    local -a want=()
    read -r -a want <<<"$expect"

    for ((i = 0; i < retries; i++)); do
        # 真失败优先判定：任一节点进程已退出 → 立刻返回 1。
        dead="$(probe_dead_nodes)"
        if [ -n "$dead" ]; then
            PROBE_DEAD_NODES="$dead"
            return 1
        fi

        # 辅助确认：daemon 视图。
        nodes="$(timeout 12 bash -c "${PROBE_NODE_LIST_CMD}" 2>/dev/null)"
        missing=0
        for n in "${want[@]}"; do
            grep -q "$n" <<<"$nodes" || missing=1
        done
        [ "$missing" -eq 0 ] && return 0

        sleep "$nap"
    done

    # 重试用尽：再看一次进程存活，区分"真失败"与"环境性失败"。
    dead="$(probe_dead_nodes)"
    if [ -n "$dead" ]; then
        PROBE_DEAD_NODES="$dead"
        return 1
    fi
    PROBE_LAST_NODE_VIEW="$nodes"
    return 2
}

# 环境性失败时给调用方的统一提示（首行必须以"疑环境性失败"开头，见 #69 判据）。
probe_env_hint() {
    echo "疑环境性失败（DDS 发现未收敛）：全部节点进程均存活，但 ros2 daemon 的节点视图为空。"
    echo "  → 请核对 CI 同 commit 结论后再改代码；本门禁继续由数据面断言判定成败。"
}
