#!/usr/bin/env bash
# ==============================================================================
# #22: Core boundary standalone check.
# Builds & runs the pure-std core unit tests WITHOUT ROS (no colcon, no
# roscore, no DDS), using the plain CMake/CTest entry at tests/core_standalone.
# ASan + UBSan are enabled to validate core memory/UB boundary safety.
# Shared by local runs and .github/workflows/ci.yml.
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${WORKSPACE_ROOT}/build/core_standalone"

echo "=== [1/3] Configure (plain CMake, no ROS, ASan+UBSan) ==="
cmake -S "${WORKSPACE_ROOT}/tests/core_standalone" \
    -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCORE_ASAN=ON

echo "=== [2/3] Build core tests (no ROS included) ==="
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo "=== [3/3] Run core tests (ctest) ==="
cd "${BUILD_DIR}"
ctest --output-on-failure

echo "✔ core standalone check passed: pure-std core build/run without ROS, ASan+UBSan clean"
