#!/usr/bin/env bash
# ==============================================================================
# Headless startup smoke check (issue #13 "无界面启动检查")
# Launches a set of hardware/GUI-free ROS 2 nodes headlessly and asserts each
# actually registers on the graph (catches: missing config, bad plugin, import
# errors, crash-on-start) that a plain build would not.
#
# Env:
#   ROS_LOCALHOST_ONLY  (forced to 1 here; runner/host-loopback only)
#   SMOKE_WAIT_SEC      per-node registration timeout (default 15)
# Exit non-zero if any expected node fails to come up.
# ==============================================================================
set -uo pipefail

export ROS_LOCALHOST_ONLY=1
WAIT_SEC="${SMOKE_WAIT_SEC:-15}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Writable temp dir for per-node logs (workspace-local so it works even when /tmp
# is read-only, e.g. under sandboxes; CI runner /tmp is fine too).
TMPD="${TMPDIR:-}"
if [ -z "${TMPD}" ] || ! mkdir -p "${TMPD}" 2>/dev/null || [ ! -w "${TMPD}" ]; then
  TMPD="${WORKSPACE_ROOT}/build/.headless_smoke"
  mkdir -p "${TMPD}" 2>/dev/null || TMPD="${WORKSPACE_ROOT}"
fi

# Each entry: "<expected-node>|<ros2 command>"
# (hardware/GUI-free C++ rclcpp nodes; no rviz, no sensor topics needed for the
#  node to construct and register on the graph.)
SMOKES=(
  "/safety_monitor|ros2 run safety_monitor safety_monitor"
  "/skidpad_planner|ros2 launch skidpad_planner skidpad_planner.launch.py"
  "/straight_line_planner|ros2 launch straight_line_planner straight_line_planner.launch.py"
  "/velocity_profiler_node|ros2 launch velocity_profiler velocity_profiler.launch.py"
  "/urinay|ros2 launch urinay urinay.launch.py"
)

fail=0
for entry in "${SMOKES[@]}"; do
  node="${entry%%|*}"
  cmd="${entry#*|}"
  proc="${node#/}"
  log="${TMPD}/smoke_${proc}.log"
  echo "--- smoke: ${cmd} (expect node ${node}) ---"
  ${cmd} > "${log}" 2>&1 &
  launcher=$!
  found=0
  for _ in $(seq 1 "${WAIT_SEC}"); do
    if ! kill -0 "${launcher}" 2>/dev/null; then
      break  # launcher died early
    fi
    if ros2 node list 2>/dev/null | grep -qx "${node}"; then
      found=1
      break
    fi
    sleep 1
  done
  # teardown: SIGINT the launch process tree, then kill leftover by node/binary name
  kill -INT "${launcher}" 2>/dev/null || true
  for _ in $(seq 1 10); do kill -0 "${launcher}" 2>/dev/null || break; sleep 0.5; done
  kill -TERM "${launcher}" 2>/dev/null || true
  wait "${launcher}" 2>/dev/null || true
  pkill -x "${proc}" 2>/dev/null || true

  if [ "${found}" -eq 1 ]; then
    echo "OK   ${node} registered"
  else
    echo "FAIL ${node} did NOT register (process died early or timed out)"
    echo "----- log: ${cmd} -----"; cat "${log}"; echo "-----------------------------"
    fail=1
  fi
  rm -f "${log}"
done

if [ "${fail}" -ne 0 ]; then
  echo "Headless startup smoke: FAILED"
  exit 1
fi
echo "Headless startup smoke: all ${#SMOKES[@]} nodes registered ✔"
