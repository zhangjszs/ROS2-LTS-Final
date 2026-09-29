#!/usr/bin/env bash
# ==============================================================================
# #48 调参驱动：结构化声明（config/mpc_tuning_declaration.json）的唯一执行入口。
#
# 与 #46 一次性脚本的区别：搜索空间 / 准则 / 参考配置不再硬编码在这里，全部由
# scripts/tuning_precheck.py 从声明读取；**开跑前强制过可满足性预检**——任一准则
# 对参考配置自身不成立且未显式豁免，拒绝开跑并指名该条（退出码非 0）。
#
#   bash scripts/tuning_driver.sh --precheck-only   # 只跑预检（秒级到分钟级）
#   bash scripts/tuning_driver.sh --dry-run         # 预检 + 打印网格命令，不实跑
#   bash scripts/tuning_driver.sh                   # 预检 → 扫网格 → 逐条判定 → 报告
#
# 非目标：不改任何 MPC 参数；比较集评估不在本脚本职责内（#45 §8：比较命令禁止旋钮，
# 由 check 机检声明保证）。产物写 build/tuning_run/（不入库）。
# 前提：已构建（benchmark_runner 存在于 --runner 指定路径，默认
# ./build/track_benchmark/benchmark_runner）。
# ==============================================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${ROOT}" || exit 1

WORKDIR="build/tuning_run"
RUNNER="./build/track_benchmark/benchmark_runner"
DECLARATION="config/mpc_tuning_declaration.json"
METRICS_FROM=""
PRECHECK_ONLY=0
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --precheck-only) PRECHECK_ONLY=1; shift ;;
        --dry-run) DRY_RUN=1; shift ;;
        --workdir) WORKDIR="$2"; shift 2 ;;
        --runner) RUNNER="$2"; shift 2 ;;
        --declaration) DECLARATION="$2"; shift 2 ;;
        --metrics-from) METRICS_FROM="$2"; shift 2 ;;
        -h|--help)
            sed -n '2,20p' "${BASH_SOURCE[0]}"
            exit 0 ;;
        *)
            echo "未知参数：$1（--help 看用法）" >&2
            exit 2 ;;
    esac
done

PRE="python3 ${SCRIPT_DIR}/tuning_precheck.py"
mkdir -p "${WORKDIR}" || { echo "无法创建 ${WORKDIR}" >&2; exit 1; }
# RUNNER 可以是多词命令（如 ros2 run track_benchmark benchmark_runner），拆成数组执行
read -ra RUNNER_ARR <<<"${RUNNER}"

echo "== [1/3] 预检（声明结构 / 参考配置对账 / 比较集无旋钮 / 准则可满足性） =="
EXTRA=()
if [ -n "${METRICS_FROM}" ]; then EXTRA+=(--metrics-from "${METRICS_FROM}"); fi
# shellcheck disable=SC2086
if ! ${PRE} check --declaration "${DECLARATION}" --workdir "${WORKDIR}" --runner "${RUNNER}" ${EXTRA[@]+"${EXTRA[@]}"}; then
    echo "预检未通过 → 拒绝开跑（这是 #48 的本意，不要绕过它直接跑网格）。" >&2
    exit 1
fi
if [ "${PRECHECK_ONLY}" -eq 1 ]; then
    echo "--precheck-only：到此为止。"
    exit 0
fi

echo
echo "== [2/3] 从声明枚举调参网格（组合 × 场景） =="
# shellcheck disable=SC2086
${PRE} commands --declaration "${DECLARATION}" >"${WORKDIR}/commands.tsv" || exit 1
total=$(wc -l <"${WORKDIR}/commands.tsv")
echo "共 ${total} 行（每组合 × 调参集场景数）；产物目录：${WORKDIR}/combos/<cid>/"
if [ "${DRY_RUN}" -eq 1 ]; then
    cat "${WORKDIR}/commands.tsv"
    echo "--dry-run：预检已通过且网格可枚举，未实跑（实跑请去掉 --dry-run）。"
    exit 0
fi

# 逐行执行：cid ⇥ scenario_key ⇥ 参数标签 ⇥ runner 参数（IFS 只对 read 生效，
# 循环体内保持默认空白分词，${args} 才能拆成多个 argv）。
# incomplete/未达门限的运行 rc≠0 是**数据**不是事故（判据看 JSON），tolerate。
while IFS=$'\t' read -r cid key label args; do
    [ -n "${cid}" ] || continue
    dir="${WORKDIR}/combos/${cid}"
    mkdir -p "${dir}"
    if [ -f "${dir}/${key}.kpi.json" ] && [ -f "${dir}/.label" ]; then
        echo "skip ${cid}/${key}（已有产物；删掉 ${dir} 可强制重跑）"
        continue
    fi
    echo "${label}" >"${dir}/.label"
    # shellcheck disable=SC2086
    "${RUNNER_ARR[@]}" ${args} --out "${dir}/${key}.kpi.json" --diag-out "${dir}/${key}.diag.json" \
        >"${dir}/${key}.out" 2>"${dir}/${key}.log"
    rc=$?
    echo "done ${cid}/${key} rc=${rc}（非 0 仅代表未达门限，产物仍是判定依据）"
done <"${WORKDIR}/commands.tsv"

echo
echo "== [3/3] 逐条准则判定 + 平手排序 =="
# shellcheck disable=SC2086
${PRE} evaluate --declaration "${DECLARATION}" --workdir "${WORKDIR}"
echo
echo "提示：本轮结论只说明'该搜索空间在新准则下的取舍'；参数变更须另行提交，"
echo "并在冻结后在保留比较集上做**一次**评估（命令不得含调参旋钮，#45 §8）。"
