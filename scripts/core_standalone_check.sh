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

echo "=== [1/4] Coverage audit: 每个纯 std 单测要么已注册，要么写进排除清单（#40）==="
# 防止“新增一个共用层单测但默默不进 sanitizer 门”——#38 那类 UB（std::clamp lo>hi）
# 只有进了本门才会被自动抓。排除原因写在 CMakeLists 的排除清单注释里，同样算“已交代”。
CMAKE_LISTS="${WORKSPACE_ROOT}/tests/core_standalone/CMakeLists.txt"
undeclared=0
declared=0
while IFS= read -r f; do
    base="$(basename "${f}" .cpp)"
    if grep -qF "${base}" "${CMAKE_LISTS}"; then
        declared=$((declared + 1))
    else
        echo "✘ ${f#"${WORKSPACE_ROOT}/"}：既没注册进 ASan+UBSan 门，也没写进排除清单" >&2
        undeclared=1
    fi
done < <(find "${WORKSPACE_ROOT}/src" -path '*/test/*' -name 'test_*.cpp' | sort)
if [ "${undeclared}" -ne 0 ]; then
    echo "处置：能脱 ROS 编译就 add_core_test(...)，不能就写清排除原因（见 #40）。" >&2
    exit 1
fi
echo "✔ 覆盖审计通过：${declared} 个单测文件均已交代（注册或列入排除清单）"

echo "=== [2/4] Configure (plain CMake, no ROS, ASan+UBSan) ==="
cmake -S "${WORKSPACE_ROOT}/tests/core_standalone" \
    -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCORE_ASAN=ON

echo "=== [3/4] Build core tests (no ROS included) ==="
cmake --build "${BUILD_DIR}" --parallel "$(nproc)"

echo "=== [4/4] Run core tests (ctest) ==="
cd "${BUILD_DIR}"
ctest --output-on-failure

echo "✔ core standalone check passed: pure-std core build/run without ROS, ASan+UBSan clean"
