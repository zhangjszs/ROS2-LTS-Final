#!/usr/bin/env bash
# ==============================================================================
# C++20 Formatting & Static Analysis (no build)
# Shared by scripts/check_cpp20.sh (local) and .github/workflows/ci.yml (CI).
# Fails (non-zero exit) on any clang-format violation or cppcheck error.
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [1/2] Checking C++20 code formatting with clang-format ==="
if command -v clang-format &> /dev/null; then
    find "${WORKSPACE_ROOT}/src" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) | xargs clang-format --dry-run --Werror
    echo "✔ clang-format passed with 0 formatting violations!"
else
    echo "⚠ clang-format is not installed. Skipping format check."
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
    echo "⚠ cppcheck is not installed. Skipping cppcheck scan."
fi
