#!/usr/bin/env bash
# ==============================================================================
# C++20 Compliance & Static Analysis Check Script
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=== [1/2] Formatting & static analysis (delegated to lint_cpp.sh) ==="
bash "${SCRIPT_DIR}/lint_cpp.sh"

echo "=== [2/2] Checking compilation with -std=c++20 and strict flags ==="
cd "${WORKSPACE_ROOT}"
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON

echo "=== All C++20 Compliance Checks Passed! ==="
