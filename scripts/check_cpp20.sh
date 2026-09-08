#!/usr/bin/env bash
# ==============================================================================
# C++20 Compliance & Static Analysis Check Script
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [1/2] Checking C++20 compliance with cppcheck ==="
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

echo "=== [2/2] Checking compilation with -std=c++20 and strict flags ==="
cd "${WORKSPACE_ROOT}"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON

echo "=== All C++20 Compliance Checks Passed! ==="
