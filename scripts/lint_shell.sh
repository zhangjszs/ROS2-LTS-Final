#!/usr/bin/env bash
# ==============================================================================
# Shell 门禁脚本的静态检查（issue #41）
#
# 为什么需要：仓库的验收判据有相当一部分是 shell（benchmark_regression / 三条闭环冒烟 /
# qos_contract / headless_smoke …），它们一旦被改出语法错误，最坏结果不是"报错"而是
# 判据根本不再执行 → 门禁假绿。C++ 有 lint_cpp.sh，shell 此前没有任何机检。
#
# 三段：
#   [1/3] bash -n 语法检查 —— **必查**，无文件可查也算失败（防止 glob 失配后空转变绿）。
#   [2/3] 断言型查询不得走 `--no-daemon` —— **必查**（#49）。
#         `ros2 node list --no-daemon` / `ros2 topic info --no-daemon` 在新进程里从零做
#         DDS 发现并**立即**查图，常常在收到周期性 SPDP 公告前就返回空表 → 门禁假失败。
#         本仓已因此判红过主干（push 36582919764、nightly 36494908622）；契约写在
#         drive_gates.sh 注释里但只落了文档，这一节把它变成机检。
#   [3/3] shellcheck —— **建议性**：存在则报告告警数，缺失则明确打印跳过原因；
#         只有传 --strict 时才因告警改变退出码（默认不因版本/规则差异卡门禁）。
#
# 用法： bash scripts/lint_shell.sh [--strict]
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

STRICT=0
for arg in "$@"; do
    case "${arg}" in
        --strict) STRICT=1 ;;
        *)
            echo "用法: bash scripts/lint_shell.sh [--strict]" >&2
            exit 2
            ;;
    esac
done

# 收集待查脚本：scripts/*.sh + scripts/git-hooks/*（钩子无扩展名，容易被漏掉）
files=()
while IFS= read -r f; do
    files+=("${f}")
done < <(
    {
        find "${WORKSPACE_ROOT}/scripts" -type f -name '*.sh'
        find "${WORKSPACE_ROOT}/scripts/git-hooks" -type f 2>/dev/null
    } | sort
)

echo "=== [1/3] bash -n syntax check (mandatory) ==="
if [ "${#files[@]}" -eq 0 ]; then
    # 取不到待查文件 = 本门失去意义，必须红，不能绿
    echo "✘ FAIL: scripts/ 下找不到任何 .sh 或 git-hook 脚本（目录结构变了就同步改本脚本）" >&2
    exit 1
fi

syntax_bad=0
for f in "${files[@]}"; do
    if ! out="$(bash -n "${f}" 2>&1)"; then
        echo "✘ ${f}"
        printf '%s\n' "${out}" | sed 's/^/    /'
        syntax_bad=1
    fi
done
if [ "${syntax_bad}" -ne 0 ]; then
    echo "bash -n: ${#files[@]} 个脚本中发现语法错误 ✘" >&2
    exit 1
fi
echo "✔ bash -n passed: ${#files[@]} shell scripts clean"

echo "=== [2/3] 断言型 ros2 查询必须走 daemon（#49，mandatory） ==="
# 只看真实执行行：排除注释行与纯 echo/printf 行（写文档/报错文案时可以提到这个选项）。
no_daemon_bad=0
for f in "${files[@]}"; do
    hits="$(sed -e 's/[[:space:]]#.*$//' "${f}" \
            | grep -vE '^[[:space:]]*#' \
            | grep -vE '^[[:space:]]*(echo|printf)\b' \
            | grep -nE '\bros2[[:space:]]+(node|topic|service|interface|action)[[:space:]]+[^|;&]*--no-daemon' || true)"
    if [ -n "${hits}" ]; then
        echo "✘ ${f}: 断言型查询用了 --no-daemon（会空表假失败，改用 daemon 并在脚本头部 ros2 daemon stop）"
        printf '%s\n' "${hits}" | sed 's/^/    /'
        no_daemon_bad=1
    fi
done
if [ "${no_daemon_bad}" -ne 0 ]; then
    echo "#49 机检：发现 --no-daemon 断言查询 ✘" >&2
    exit 1
fi
echo "✔ 未发现 --no-daemon 断言型查询（${#files[@]} 个脚本）"

echo "=== [3/3] shellcheck (advisory) ==="
if ! command -v shellcheck >/dev/null 2>&1; then
    echo "⚠ shellcheck 未安装，跳过建议性检查（CI 的 apt 步骤已安装它；本地装法：apt-get install shellcheck）"
    exit 0
fi

sc_issues=0
for f in "${files[@]}"; do
    if ! shellcheck --severity=warning "${f}"; then
        sc_issues=1
    fi
done

if [ "${sc_issues}" -ne 0 ]; then
    if [ "${STRICT}" -eq 1 ]; then
        echo "shellcheck: 存在 warning 级问题，--strict 下判失败 ✘" >&2
        exit 1
    fi
    echo "⚠ shellcheck 报了 warning 级建议（不卡门禁；需要卡时加 --strict）"
else
    echo "✔ shellcheck passed with 0 warnings (severity>=warning)"
fi
exit 0
