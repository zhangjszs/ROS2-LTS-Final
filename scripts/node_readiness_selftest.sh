#!/usr/bin/env bash
# ==============================================================================
# #69：就绪探测共用件 `lib/node_readiness.sh` 的**行为自测**。
#
# 为什么需要自测（Issue #69 验收判据 1/3）：
#   判据 1 要求"节点全部存活但 daemon 视图为空"的**可构造**情形；
#   判据 3 要求"人为让某个节点进程立刻退出"时门禁必须非 0 退出并点名节点。
#   这两个情形在真实 CI 里都是偶发的（run 40205834746 只中了一次），靠运气
#   复现等于没有回归保护，因此这里用假进程 + 注入的节点视图查询来**确定性地**
#   构造它们。
#
# 三条断言：
#   A) 进程全活 + 视图含全部节点  → 返回 0（就绪）
#   B) 进程全活 + 视图为空        → 返回 2（**环境性**：不得判红）
#   C) 某节点进程已退出           → 返回 1（**真失败**：必须判红并点名该节点）
#
# 纯 bash，无 ROS 依赖，<1s；由 lint_shell.sh 顺带执行（不改 drive_gates 门清单）。
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/lib/node_readiness.sh
source "${SCRIPT_DIR}/lib/node_readiness.sh"

fails=0

expect_rc() {  # expect_rc <期望返回码> <实际返回码> <用例名>
    if [ "$1" -ne "$2" ]; then
        echo "  ✘ $3：期望返回码 $1，实测 $2"
        fails=$((fails + 1))
    else
        echo "  ✔ $3：返回码 $2"
    fi
}

# ── A) 进程全活 + 视图完整 → 就绪 ────────────────────────────────────────────
sleep 300 &
p1=$!
sleep 300 &
p2=$!
PROBE_NODE_NAMES=()
PROBE_NODE_PIDS=()
probe_register "alpha_node" "$p1"
probe_register "beta_node" "$p2"
PROBE_NODE_LIST_CMD="printf 'alpha_node\nbeta_node\n'"
probe_wait_ready "alpha_node beta_node" 2 0
expect_rc 0 "$?" "A 进程全活且视图完整 → 就绪"

# ── B) 进程全活 + 视图为空 → 环境性（不得判红）────────────────────────────────
PROBE_NODE_LIST_CMD="true"   # 输出空 → 模拟 daemon 视图空表
probe_wait_ready "alpha_node beta_node" 2 0
rc=$?
expect_rc 2 "$rc" "B 进程全活但视图为空 → 环境性（不判红）"
if [ "$rc" -eq 2 ] && [ -z "$PROBE_DEAD_NODES" ]; then
    echo "  ✔ B 未误报节点死亡（PROBE_DEAD_NODES 为空）"
else
    echo "  ✘ B 误报节点死亡：'${PROBE_DEAD_NODES}'"
    fails=$((fails + 1))
fi

# ── C) 某节点进程立刻退出 → 真失败（必须判红并点名）────────────────────────────
kill "$p2" 2>/dev/null
wait "$p2" 2>/dev/null
sleep 300 &
p3=$!
PROBE_NODE_NAMES=()
PROBE_NODE_PIDS=()
probe_register "alpha_node" "$p1"
probe_register "dead_node" "$p3"
kill "$p3" 2>/dev/null
wait "$p3" 2>/dev/null
PROBE_NODE_LIST_CMD="true"
probe_wait_ready "alpha_node dead_node" 2 0
rc=$?
expect_rc 1 "$rc" "C 节点进程已退出 → 真失败（判红）"
if [ "$rc" -eq 1 ] && grep -qx "dead_node" <<<"$PROBE_DEAD_NODES"; then
    echo "  ✔ C 正确点名未起来的节点：dead_node"
else
    echo "  ✘ C 未正确点名未起来的节点：'${PROBE_DEAD_NODES}'"
    fails=$((fails + 1))
fi

kill "$p1" 2>/dev/null
wait "$p1" 2>/dev/null

if [ "$fails" -ne 0 ]; then
    echo "node_readiness selftest: ${fails} 条断言失败 ✘" >&2
    exit 1
fi
echo "node_readiness selftest: 全部断言通过 ✔"
