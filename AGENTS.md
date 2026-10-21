# AGENTS.md

Full project context (architecture, data flow, package roles) lives in **[CLAUDE.md](CLAUDE.md)**. Read its "Local Validation Loop" section before editing. Prior agent sessions leave continuity state in `.agent/` — check `.agent/HANDOFF.md` first if it exists.

## Build

- `colcon build --symlink-install` picks up `colcon_defaults.yaml` (mold linker, ccache, `parallel-workers: 4`, `BUILD_TESTING=OFF`). Do NOT raise `parallel-workers` — 24 threads on 19 GB RAM swap-thrashes.
- A CLI `--cmake-args` list **replaces** the file's list, so repeat mold/ccache every time (see `ci.yml` build step for the exact list). Single package: `colcon build --packages-select <name>`.
- Always exclude the test-less packages: `--packages-ignore hello_world huat_launch` (demo / launch-only).
- `tests/` has `COLCON_IGNORE` — the plain-CMake core suite is never built by colcon (see Tests).

## Validate (mirrors CI — run this, don't improvise)

- One entry point: `bash scripts/drive_gates.sh` (~10 min). `QUICK=1` = lint + no-ROS core only (seconds, run after every edit batch). `SKIP_BUILD=1` = runtime gates on an existing build.
- Do NOT write ad-hoc verification when a `scripts/*.sh` gate already covers it — extend the script so the fix lands in CI too.
- `bash scripts/lint_cpp.sh`: fails if clang-format/cppcheck are missing (bypass only via `ALLOW_MISSING_TOOLS=1`, with a written reason). `bash scripts/lint_shell.sh`: `bash -n` is mandatory; shellcheck is advisory unless `--strict`.
- `colcon test --packages-ignore hello_world huat_launch && colcon test-result --verbose` — the `test-result` step is what actually fails the build on test failures.

## Runtime env (local ≠ CI)

- Local ROS distro is **lyrical** (`/opt/ros/lyrical`); CI runs **Jazzy**. Source both workspaces for runtime gates: `source /opt/ros/<distro>/setup.bash && source install/setup.bash`.
- Multi-node gates need `export FASTDDS_BUILTIN_TRANSPORTS=SHM` locally (local UDP-multicast discovery is unreliable behind VPN; not needed on CI).
- `drive_gates.sh` already points `ROS_HOME`/`TMPDIR` into `build/` — don't override; a read-only `/tmp` prevents the ros2 daemon from starting.

## ros2 queries: always via daemon

- Assertion queries (`ros2 node|topic|service list|info`) must use the ros2cli daemon. Never `--no-daemon` — it returns empty tables before DDS discovery converges (machine-checked by `lint_shell.sh`, past main-branch reds).
- Gate scripts start with `timeout 20 ros2 daemon stop` → **explicit `ros2 daemon start`** → `sleep 3`. Do NOT assume later calls rebuild the daemon automatically: in fresh per-session environments it dies with its parent and assertion queries hang with `rc=124` (#82; `drive_gates.sh` already does this).

## Tests: Core/ROS boundary (ADR: `docs/CORE_ROS_BOUNDARY_ADR.md`)

- New algorithm logic goes in **Core**: header-only, C++20 stdlib only — no `rclcpp`, no generated message headers. ROS stays in the adapter layer; `concept` duck-typing bridges them.
- Every `src/**/test/test_*.cpp` must be accounted for in `tests/core_standalone/CMakeLists.txt`: `add_core_test(...)` if it compiles without ROS, otherwise a reason in the exclusion list — else `core_standalone_check.sh` (ASan+UBSan, no ROS/DDS) fails.
- Contract topics (`vehicle_state`, `pathlimits`, `vehicle_command`, `stop`, `/system/state`): build QoS only via `common_msgs::contract::makeQoS(spec)` (`src/common/common_msgs/include/common_msgs/interface_contract_qos.hpp`, spec in `interface_contract.h`). Publishing to a latched topic with default (volatile) QoS silently delivers nothing. Topic/QoS changes must update `docs/INTERFACE_CONTRACT.md` + `scripts/qos_contract_check.sh` together.

## CMake / C++20

- Target-based only: `common_msgs::common_msgs`, `rclcpp::rclcpp`. Forbidden: `${rclcpp_LIBRARIES}`, `${pkg}_INCLUDE_DIRS`, recompiling business `.cpp` sources inside tests (link the lib).
- `.clang-format`: Google base, 4-space, 120 cols. When touching code, prefer `<numbers>`, designated initializers, `std::format`/`span`/`ranges`, `<=>`, `std::jthread` over legacy equivalents.
- Fast format gate for staged files: `cp scripts/git-hooks/pre-commit .git/hooks/ && chmod +x .git/hooks/pre-commit`.

## Tuning (frozen protocol)

- Only entry: `bash scripts/tuning_driver.sh` (precheck: `python3 scripts/tuning_precheck.py selftest`); declaration: `config/mpc_tuning_declaration.json`. Never loosen criteria after seeing results — new rounds need a re-declared round + pre-registration (`docs/MPC_TUNING_FREEZE.md`).

## Issues

- Tracker: `zhangjszs/ROS2-LTS-Final` via `gh`. Agent-ready label is `ready-for-agent`; human work is `ready-for-human` (map in `docs/agents/triage-labels.md`).
