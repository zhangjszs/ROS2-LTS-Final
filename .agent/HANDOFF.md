# HANDOFF
## 本轮概要
- `exec-20261008T1123Z`：领取 #57（ready 队列唯一项）一次做完——新增 `scripts/demo.sh`（一键正常/故障仿真演示）+ README 展示小节；本机两臂实跑 + 两次失败演练全绿，push CI success → 执行报告已留 → 转 `in-review`。ready 队列清空，正常收尾。

## 已完成
- #57：`c79b71d`（merge `97de15d`，已推送）· 六条验收逐项满足 · `bash scripts/demo.sh` rc=0（正常臂 13s / 故障臂 46s）+ `QUICK=1 drive_gates.sh` 5 门全 PASS + push CI `37770836888` success · 执行报告见 Issue #57 comment。

## 未完成 / 进行中（下一棒最优先看这里）
- 无 in-progress。#57 为 `in-review` 等 Planner 验收；#26/#25 维持 needs-info（完整展示 / 博客，属 Planner 侧）。
- 若 Planner 无新 ready：只做观察——查 Nightly（`gh run list --workflow=nightly -L 3`），出现新红按新根因建账，不要重开 #55。

## 验证情况
- 跑了：`bash -n scripts/demo.sh`（rc=0）· `bash scripts/lint_shell.sh`（rc=0，15 个脚本）· `bash scripts/demo.sh`（rc=0，两臂 PASS + 故障臂证据核对通过）· 演练 A（两臂 stub 必败 → rc=1，两条失败项都列出）· 演练 B（故障臂 stub 成 `true` → 证据核对拦下缺 JSON，rc=1）· `QUICK=1 bash scripts/drive_gates.sh`（lint/cpp20/core-standalone/benchmark/tuning-precheck 全 PASS，22s）· push CI `37770836888`（main `97de15d`）success。
- 未跑：全量 `drive_gates.sh`（本改动只新增聚合入口脚本 + README；QUICK 的 lint 门已覆盖新脚本，两臂脚本本身在 demo 与 QUICK 链路上均已实跑）。

## 风险与注意事项
- `M AGENTS.md` 脏改动开工即在（非本体系产生）：全程未碰、未提交，下一棒同样只读。
- 本机跑多节点脚本必须 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`：本轮实测不设时 `ros2 node list` 空表 → 闭环冒烟假红（报"闭环节点未全部注册"）。
- `scripts/demo.sh` 前置 = 已构建（`install/setup.bash` 存在），否则明确 FAIL 并非零退出；`OUT_DIR` 可用环境变量改（默认 `build/demo`）。
- `DEMO_NORMAL_CMD` / `DEMO_FAULT_CMD` 是失败演练钩子（覆盖整条臂命令），日常不要使用。

## 给下一棒的第一步建议
1. 无新 ready 时：`git fetch` 后确认 main == origin/main（注意 `AGENTS.md` 脏改动仍在，勿动它、勿 `reset --hard`），看一眼 Nightly，无异常即收尾；本轮无需恢复现场。
2. 有新 ready 时：按 2.3 顺序领取；验证前先 `export PATH="$HOME/.local/bin:$PATH"`，多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。

## 给 Planner 的信号
- #57 可验收（逐条证据见 Issue #57 执行报告）。ready 队列本轮清空；无阻塞、无新增 auto-discovered（配额 0/3）。
- 备注（非问题、未建账）：`CLAUDE.md` 的冒烟脚本清单未列 `scripts/demo.sh`——它是聚合入口而非新门禁，本轮按"最小改动、不扩范围"未改该文档；若 Planner 认为需要，可另立文档类 Issue。
