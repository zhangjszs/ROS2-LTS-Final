#!/usr/bin/env bash
# ==============================================================================
# C++20 Formatting & Static Analysis (no build)
# Shared by scripts/check_cpp20.sh (local) and .github/workflows/ci.yml (CI).
# Fails (non-zero exit) on any clang-format violation or cppcheck error.
#
# 工具缺失不得静默变绿（#41）：clang-format / cppcheck 缺失时本门直接判失败，
# 否则“本地没装工具”会伪装成“代码格式干净”。CI 的 apt 步骤两个都装，不受影响。
# 确实无法安装的环境可显式绕过（并在评审时说明）：ALLOW_MISSING_TOOLS=1。
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ALLOW_MISSING_TOOLS="${ALLOW_MISSING_TOOLS:-0}"

missing_tools=0

echo "=== [1/2] Checking C++20 code formatting with clang-format ==="
if command -v clang-format &> /dev/null; then
    find "${WORKSPACE_ROOT}/src" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) | xargs clang-format --dry-run --Werror
    echo "✔ clang-format passed with 0 formatting violations!"
else
    echo "⚠ clang-format 未安装" >&2
    missing_tools=1
fi

echo "=== [2/2] Checking C++20 compliance with cppcheck ==="
if command -v cppcheck &> /dev/null; then
    cppcheck \
        --language=c++ \
        --std=c++20 \
        --enable=warning,performance,portability \
        --inline-suppr \
        -q \
        -I "${WORKSPACE_ROOT}/src" \
        $(find "${WORKSPACE_ROOT}/src" -name "*.cpp" -o -name "*.h" -o -name "*.hpp")
    echo "✔ cppcheck passed with 0 issues!"
else
    echo "⚠ cppcheck 未安装" >&2
    missing_tools=1
fi

if [ "${missing_tools}" -ne 0 ] && [ "${ALLOW_MISSING_TOOLS}" != "1" ]; then
    echo "✘ 静态检查工具缺失，本门不得在“未检查”状态下判绿。" >&2
    echo "  安装：apt-get install clang-format cppcheck   （或 ALLOW_MISSING_TOOLS=1 显式跳过并说明）" >&2
    exit 1
fi
