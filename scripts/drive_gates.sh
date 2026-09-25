#!/usr/bin/env bash
# ==============================================================================
# 一条命令跑完整门禁（本地与 CI 同一套判据），供人、cron、agent 复用。
#
#   bash scripts/drive_gates.sh              # 全跑（含构建，慢）
#   SKIP_BUILD=1 bash scripts/drive_gates.sh # 只跑运行时门禁（已构建过时用）
#   QUICK=1     bash scripts/drive_gates.sh # lint + 无 ROS 独立检查（秒级，改完就跑）
#
# 退出码只反映**硬门禁**；三条容差型集成门禁（闭环 / 闭环故障样本 / 故障注入）按 CI 语义记为观察项，
# 红不翻转退出码，但会打印 [OBSERVE] 并计入 summary，供"观察期转硬门禁"积累证据。
# 结果同时写成 build/drive_gates/last.json + 追加 build/drive_gates/history.{log,jsonl}，
# 让任何定时机制（GitHub Actions cron / CI / agent）都读同一份证据而不是各自重述。
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}" || exit 1

OUT=build/drive_gates
mkdir -p "${OUT}"
: "${ROS_HOME:=${ROOT}/build/.ros}"
export ROS_HOME
mkdir -p "${ROS_HOME}"
# ros2cli daemon 的 socket 落在 tempfile.gettempdir()（默认 /tmp）。只读 /tmp 的环境里
# daemon 根本起不来，`ros2 node list` 会返回空表、把全部节点误判为"未注册"。
# 指到工作区内可写目录，而不是让各门禁脚本去绕 daemon。
: "${TMPDIR:=${ROOT}/build/tmp}"
export TMPDIR
mkdir -p "${TMPDIR}"

# 运行时门禁靠 ros2cli daemon 持续维护的图（`ros2 topic info --no-daemon` 会从零发现、
# 端点未收敛就返回 0 —— 已在 Jazzy CI 实测导致误报，故不得改走 --no-daemon）。
# 残留/空转的 daemon 会让 `ros2 node list` 返回空表、把全部节点误判为未注册；
# 这里先停一次，后续调用会自动按当前 TMPDIR/ROS_HOME 重建（CI 上无旧 daemon，等于空操作）。
timeout 20 ros2 daemon stop >/dev/null 2>&1 || true

started_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
t0=$(date +%s)
if [ -z "${FASTDDS_BUILTIN_TRANSPORTS:-}" ]; then
    echo "note: FASTDDS_BUILTIN_TRANSPORTS 未设置；本机若 UDP 组播被 VPN 挡住，"
    echo "      运行时门禁可能因 DDS 发现失败而红（CI 上无需设置）。"
fi

# ROS 环境：调用方已 source 就沿用，否则尽力自行 source。
# 关键：colcon/ROS 生成的 setup.sh 引用未定义变量，在 `set -u` 下直接终止本脚本
# （实测：source install/setup.bash 后立刻 rc=1 且零输出）。所以 source 必须临时关 nounset。
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
if [ "${SKIP_SOURCE_WS:-0}" != 1 ]; then
    source_sh install/setup.bash
fi

# 全新 checkout 上构建前 install/ 还不存在，上面那次 source 会静默跳过；
# 因此构建完成后必须再 source 一次，否则后续 `ros2 run <pkg>` 全部报"Package not found"。
source_workspace() { source_sh install/setup.bash; }

HARD_FAIL=0
declare -a ROWS=()

# 与 CI 完全同一入口：hello_world（ament_python demo）与 huat_launch（纯 launch）无 C++ 测试。
IGNORE_PKGS="${IGNORE_PKGS:-hello_world huat_launch}"

# run_gate <name> <hard:1|0> <cmd...>
run_gate() {
    local name="$1" hard="$2"
    shift 2
    local log="${OUT}/${name}.log" t_start t_end rc status line
    t_start=$(date +%s)
    printf '%-26s' "$name"
    "$@" >"${log}" 2>&1
    rc=$?
    t_end=$(date +%s)
    if [ "${rc}" -eq 0 ]; then
        status="PASS"
    elif [ "${hard}" -eq 1 ]; then
        status="FAIL"
        HARD_FAIL=1
    else
        status="OBSERVE"
    fi
    line=$(( t_end - t_start ))
    echo "${status} (${line}s)"
    [ "${rc}" -ne 0 ] && tail -n 5 "${log}" | sed 's/^/    | /'
    ROWS+=("${name}|${hard}|${status}|${line}|${rc}")
    return 0
}

if [ "${QUICK:-0}" != 1 ]; then
    if [ "${SKIP_BUILD:-0}" != 1 ]; then
        # 注意：CLI 的 --cmake-args 会整体替换 colcon_defaults.yaml 的同名列表，
        # 因此 mold/ccache 必须重列（与 .github/workflows/ci.yml 逐项一致）。
        run_gate build 1 colcon build --symlink-install --packages-ignore ${IGNORE_PKGS} --cmake-args \
            -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD \
            -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON
        run_gate test-run 1 colcon test --packages-ignore ${IGNORE_PKGS}
        run_gate test-enforce 1 colcon test-result --verbose
        source_workspace
    fi
fi

run_gate lint 1 bash scripts/lint_cpp.sh
run_gate cpp20 1 bash scripts/check_cpp20.sh
run_gate core-standalone 1 bash scripts/core_standalone_check.sh
run_gate benchmark 1 bash scripts/benchmark_regression.sh
if [ "${QUICK:-0}" != 1 ]; then
    run_gate headless-smoke 1 bash scripts/headless_smoke.sh
    run_gate qos-contract 1 bash scripts/qos_contract_check.sh
    run_gate closed-loop 0 bash scripts/closed_loop_sim_smoke.sh
    run_gate closed-loop-fault 0 bash scripts/closed_loop_fault_smoke.sh
    run_gate fault-injection 0 bash scripts/fault_injection_smoke.sh
fi

elapsed=$(( $(date +%s) - t0 ))

# 机读结果（#17/#16 的观察窗口靠这份文件累计证据，而不是靠对话记忆）
{
    printf '{\n  "schema": "fsac.drive_gates/v1",\n  "started_at": "%s",\n  "elapsed_s": %d,\n  "hard_fail": %d,\n  "gates": [\n' \
        "${started_at}" "${elapsed}" "${HARD_FAIL}"
    for i in "${!ROWS[@]}"; do
        IFS='|' read -r name hard status secs rc <<<"${ROWS[$i]}"
        printf '    {"name": "%s", "hard": %s, "status": "%s", "seconds": %s, "rc": %s}%s\n' \
            "${name}" "${hard}" "${status}" "${secs}" "${rc}" "$([ "$i" -lt $((${#ROWS[@]} - 1)) ] && echo , || true)"
    done
    printf '  ]\n}\n'
} >"${OUT}/last.json"

{
    echo "${started_at}"
    for r in "${ROWS[@]}"; do echo "  $r"; done
    echo "  elapsed=${elapsed}s hard_fail=${HARD_FAIL}"
} >>"${OUT}/history.log"

# 机读历史（一行一次运行）：供“观察期何时可转硬门禁”直接数连续绿。
{
    printf '{"started_at":"%s","elapsed_s":%d,"hard_fail":%d,"gates":{' \
        "${started_at}" "${elapsed}" "${HARD_FAIL}"
    first=1
    for r in "${ROWS[@]}"; do
        IFS='|' read -r name hard status secs rc <<<"$r"
        [ "$first" -eq 1 ] && first=0 || printf ','
        printf '"%s":"%s"' "${name}" "${status}"
    done
    printf '}}\n'
} >>"${OUT}/history.jsonl"

if [ "${HARD_FAIL}" -eq 0 ]; then
    echo "drive_gates: 硬门禁全部通过（${elapsed}s）；观察项见 ${OUT}/last.json"
else
    echo "drive_gates: 有硬门禁失败（${elapsed}s）——见 ${OUT}/*.log"
fi
exit "${HARD_FAIL}"
