#!/usr/bin/env bash
# ==============================================================================
# #57：最小可复现展示入口 —— 一条命令看清系统"能跑什么"，全程无界面。
#
#   bash scripts/demo.sh
#
# 两臂（都是既有 CI 常绿门禁脚本，本入口只负责串联、回显与退出码传播）：
#   正常臂 scripts/closed_loop_sim_smoke.sh
#     多节点闭环真的闭合：/clock 按仿真时间推进、车动起来、出口指令帧合法（#15 帧契约）、
#     速度载体与评测器都接上（详细判据见该脚本头部）。
#   故障臂 scripts/fault_injection_smoke.sh
#     注入软故障（路径停发 / 空路径 / 篡改帧 / 外部停车 / 任务锁存停 / 监控退出 / 仲裁器重启…），
#     自动判定"检测 → 状态迁移 → 制动输出"的终态，并把时间线落盘成 JSON。
#
# 退出码：任一臂非零 → 本脚本非零，绝不"跑完就算过"。
#   故障臂除 rc 外还从 build/demo/fault/*.json 抽取检测/状态/制动证据并回显；
#   证据文件缺失或字段不符同样判失败（#57 验收标准 3 要求可观察证据，不只看退出码）。
#
# 失败演练（#57 验收标准 6）：两臂命令可用环境变量替换，用于验证失败真的会传播：
#   DEMO_NORMAL_CMD=false bash scripts/demo.sh    # 正常臂被 stub 成必败 → 退出码非零
#
# 边界（D-004）：本演示只提供**仿真证据**，不构成实车可用结论。硬件急停链路、VCU 断连/
# 掉电、感知与规划质量（#18/#19）等需台架/实车或真实数据，不在本入口的证明范围内。
#
# 本机注意：若 UDP 组播被 VPN 挡住（本仓本机实测如此），先
#   export FASTDDS_BUILTIN_TRANSPORTS=SHM
# 全程无界面：不拉 RViz、不需要 DISPLAY，ssh / CI runner 上可直接跑。
# 运行前需已构建（install/setup.bash 存在）；脚本自身会 source ROS 与工作区。
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}" || exit 1

OUT_DIR="${OUT_DIR:-build/demo}"
mkdir -p "${OUT_DIR}" || { echo "FAIL: 无法创建输出目录 ${OUT_DIR}" >&2; exit 1; }

# 与各门禁脚本同一套可写目录约定：只读 /tmp 的环境里 ros2cli daemon 起不来（见 drive_gates.sh）。
export ROS_LOCALHOST_ONLY=1
: "${ROS_HOME:=${ROOT}/build/.ros}"
export ROS_HOME
mkdir -p "${ROS_HOME}"
: "${TMPDIR:=${ROOT}/build/tmp}"
export TMPDIR
mkdir -p "${TMPDIR}"

# source 必须临时关掉 nounset：ROS/colcon 生成的 setup.sh 引用了未定义变量，
# `set -u` 下会直接终止本脚本（实测：rc=1 且零输出）。
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
if ! command -v ros2 >/dev/null 2>&1; then
    echo "FAIL: 找不到 ros2 —— 先安装 ROS 2，或在已 source 过 ROS 的 shell 里运行" >&2
    exit 1
fi
if [ ! -f install/setup.bash ]; then
    echo "FAIL: 缺少 install/setup.bash —— 先构建：colcon build --symlink-install" >&2
    exit 1
fi
source_sh install/setup.bash

# 残留/空转的 ros2cli daemon 会让断言型查询恒空（各门禁同一契约）；先停一次，
# 后续调用按当前 ROS_HOME/TMPDIR 重建（CI 上本来就没有旧 daemon，等于空操作）。
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true

if [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ]; then
    echo "提示: 未设置 FASTDDS_BUILTIN_TRANSPORTS；若本机 UDP 组播被 VPN 挡住，两臂都会红。"
    echo "      本仓本机用法： export FASTDDS_BUILTIN_TRANSPORTS=SHM （CI 上不需要）"
fi

# 先清上一轮产物：否则某臂被 stub / 提前退出时，证据核对会吃到旧 JSON 而假绿。
rm -rf "${OUT_DIR:?}/normal" "${OUT_DIR:?}/fault"

ARM_CMD_NORMAL=(bash "${SCRIPT_DIR}/closed_loop_sim_smoke.sh" "${OUT_DIR}/normal")
ARM_CMD_FAULT=(bash "${SCRIPT_DIR}/fault_injection_smoke.sh" "${OUT_DIR}/fault")
# 失败演练钩子（验收标准 6）：覆盖整条臂命令，验证非零退出确实会传播到本脚本；默认不启用。
if [ -n "${DEMO_NORMAL_CMD:-}" ]; then ARM_CMD_NORMAL=(bash -c "${DEMO_NORMAL_CMD}"); fi
if [ -n "${DEMO_FAULT_CMD:-}" ]; then ARM_CMD_FAULT=(bash -c "${DEMO_FAULT_CMD}"); fi

run_arm() {   # run_arm <标签> <日志文件> <命令...>
    local label="$1" log="$2"
    shift 2
    local t0 t1 rc
    t0=$(date +%s)
    "$@" >"${log}" 2>&1
    rc=$?
    t1=$(date +%s)
    if [ "${rc}" -eq 0 ]; then
        echo "== ${label}: PASS (rc=0, $((t1 - t0))s)  日志: ${log}"
    else
        echo "== ${label}: FAIL (rc=${rc}, $((t1 - t0))s)  日志: ${log}"
        echo "   ---- 日志尾部 ----"
        tail -n 15 "${log}" | sed 's/^/   | /'
    fi
    return "${rc}"
}

FAILS=()

echo "=== 正常臂：多节点闭环仿真（回路是否真的闭合、指标是否真的在动） ==="
if run_arm "正常臂 closed_loop_sim_smoke" "${OUT_DIR}/normal.log" "${ARM_CMD_NORMAL[@]}"; then
    grep -E 'sim 时间推进|vehicle_state|vehicle_command|pathlimits|报告存在|数据面断言通过|PASS' \
        "${OUT_DIR}/normal.log" | sed 's/^/   | /'
else
    FAILS+=("正常臂 closed_loop_sim_smoke.sh")
fi

echo
echo "=== 故障臂：故障注入 → 检测 → 状态迁移 → 制动输出 ==="
if run_arm "故障臂 fault_injection_smoke" "${OUT_DIR}/fault.log" "${ARM_CMD_FAULT[@]}"; then
    grep -E '^(OK|GAP|FAIL)' "${OUT_DIR}/fault.log" | sed 's/^/   | /'
else
    FAILS+=("故障臂 fault_injection_smoke.sh")
fi

# 故障臂的机读证据（#57 标准 3）：不只看 rc，逐字段核对"检测/状态迁移/制动"真的发生。
if [ "${#FAILS[@]}" -eq 0 ]; then
    echo
    echo "--- 故障臂证据（取自 ${OUT_DIR}/fault/*.json，逐字段核对，非仅退出码） ---"
    if ! python3 - "${OUT_DIR}/fault" <<'PY'
import json
import sys
from pathlib import Path

d = Path(sys.argv[1])

TASK = {0: "IDLE", 1: "ARMED", 2: "RUNNING", 3: "FINISHED", 4: "FAULT"}
SAFETY = {0: "NORMAL", 1: "DEGRADED", 2: "STOP"}
STOPKIND = {0: "NONE", 1: "TIMEOUT", 2: "REQUEST", 3: "FAULT"}
SRC = {0: "NONE", 1: "PurePursuit", 2: "MPC"}
UNTRUSTED = {0: "NONE", 1: "ABSENT", 2: "BAD_FRAME", 3: "BAD_CHECKSUM", 4: "STALE"}

fails = []


def load(case):
    f = d / f"{case}.json"
    if not f.is_file():
        fails.append(f"缺少时间线 {f}（故障臂未落盘该场景证据）")
        return None
    try:
        return json.loads(f.read_text())
    except json.JSONDecodeError as e:
        fails.append(f"{f} 不是合法 JSON: {e}")
        return None


def check(case, cond, msg):
    if not cond:
        fails.append(f"{case}: {msg}")


# 场景 02：路径停发 → 看门狗判超时 → 状态迁移到停 → 出口零油门 + 制动
rec = load("02_path_lost")
if rec is not None:
    post = rec.get("post", {})
    print(f"[02_path_lost] 路径停发 → 检测: task={TASK.get(post.get('task_state'))} "
          f"safety={SAFETY.get(post.get('safety_state'))} stop_active={post.get('stop_active')} "
          f"来源={STOPKIND.get(post.get('stop_kind'))}")
    print(f"[02_path_lost] 制动输出: pedal={post.get('out_pedal')} brake={post.get('out_brake')} "
          f"车速={post.get('speed_mps')} m/s")
    check("02_path_lost", post.get("stop_active") is True, "路径停发后未进入停车态（未检测到）")
    check("02_path_lost", post.get("stop_kind") == 1, "停车来源不是 TIMEOUT（看门狗未触发）")
    check("02_path_lost", post.get("out_pedal") == 0, "停车后出口仍有油门")
    check("02_path_lost", (post.get("out_brake") or 0) > 0, "停车后出口没有制动")

# 场景 06：任务级 abort → FAULT 类锁存停，且监控释放也不解除
rec = load("06_abort_is_latched")
if rec is not None:
    post = rec.get("post", {})
    print(f"[06_abort_is_latched] 任务级 abort → 锁存停: stop_active={post.get('stop_active')} "
          f"来源={STOPKIND.get(post.get('stop_kind'))} task={TASK.get(post.get('task_state'))}")
    print(f"[06_abort_is_latched] 制动输出: pedal={post.get('out_pedal')} brake={post.get('out_brake')}")
    check("06_abort_is_latched", post.get("stop_active") is True, "abort 未触发锁存停")
    check("06_abort_is_latched", post.get("stop_kind") == 3, "abort 的停车来源不是 FAULT")
    check("06_abort_is_latched", post.get("out_pedal") == 0, "锁存停期间出口有油门")
    check("06_abort_is_latched", (post.get("out_brake") or 0) > 0, "锁存停期间出口没有制动")

# 场景 04：源帧被篡改 → 信任门拒绝（bad_checksum + 计入拒收），且不接管出口
rec = load("04_tampered_source_frame")
if rec is not None:
    post = rec.get("post", {})
    print(f"[04_tampered_source_frame] 篡改帧 → 判定={UNTRUSTED.get(post.get('src_b_status'))} "
          f"累计拒收={post.get('src_b_rejected')} 出口胜出源={SRC.get(post.get('winner'))} "
          f"pedal={post.get('out_pedal')}")
    check("04_tampered_source_frame", post.get("src_b_status") == 3, "篡改帧未被判为 bad_checksum")
    check("04_tampered_source_frame", (post.get("src_b_rejected") or 0) > 0, "篡改帧未被计入拒收")
    check("04_tampered_source_frame", post.get("winner") != 2, "被篡改的源接管了出口（信任门失效）")

if fails:
    print("证据核对未通过：")
    for m in fails:
        print(f"  - {m}")
    sys.exit(1)
print("证据核对通过：检测 / 状态迁移 / 制动输出均可观察（仅仿真证据）")
PY
    then
        FAILS+=("故障臂证据核对")
    fi
fi

echo
if [ "${#FAILS[@]}" -eq 0 ]; then
    echo "demo: 正常臂 + 故障臂全部 PASS —— 仅仿真证据，不构成实车可用结论（见 README 展示小节）"
    echo "demo: 产物：${OUT_DIR}/normal.log · ${OUT_DIR}/fault.log · ${OUT_DIR}/fault/*.json"
    exit 0
fi
echo "demo: FAILED —— 失败项："
for f in "${FAILS[@]}"; do echo "  - ${f}"; done
exit 1
