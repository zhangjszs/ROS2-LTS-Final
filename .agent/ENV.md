# ENV.md — 环境探测缓存（人工可直接编辑覆盖，人工版本优先于重新探测）

- 主分支：main（`git remote show origin` HEAD branch）
- 构建：`MAKEFLAGS=-j4 colcon build --symlink-install --packages-ignore hello_world huat_launch --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DCMAKE_LINKER_TYPE=MOLD -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DBUILD_TESTING=ON`（先 source ROS 与 install/setup.bash；CLI --cmake-args 会整体替换 colcon_defaults.yaml 同名列表，必须重列 mold/ccache）
- 测试：`colcon test --packages-ignore hello_world && colcon test-result --verbose`（CI 上还跑 headless_smoke / qos_contract_check / benchmark_regression / 三条容差集成门禁，见 .github/workflows/ci.yml）
- Lint：`bash scripts/lint_cpp.sh && bash scripts/lint_shell.sh`
- 全量本机门禁：`bash scripts/drive_gates.sh`（12 门，约 9–10 分钟，多节点脚本前须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`）
- gh：可用（账号 zhangjszs）
- PATH（2026-10-06T23:xxZ 实测）：agent 非交互 shell 的 PATH 缺 `~/.local/bin`，`which mold`/`which ccache` 为空系误判；本体在 `~/.local/bin/mold`（2.42.0）与 `~/.local/bin/ccache`（4.14，另有 apt 版 `/usr/bin/ccache` 4.12.3），login shell（`bash -lc`）PATH 含该目录即命中。build/验证前一律先 `export PATH="$HOME/.local/bin:$PATH"`，勿重装、勿清缓存
- 探测于 2026-09-29T23:14:07Z，agent qoder-20260929T231407Z
