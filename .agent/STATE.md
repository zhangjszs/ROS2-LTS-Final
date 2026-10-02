# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`glm-20261002T120930Z`｜时间范围：2026-10-02T12:09Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒：`glm-20261002T111615Z`（Nightly 假红根因修复，`2c9a4e6`+`a79fe83`）。
> 环境探测缓存见 `.agent/ENV.md`（人工可编辑覆盖）。

## 当前活跃

- 无（本轮收尾中）。调参腿已收口，下一棒回到 #17/#19/#15/#16 软件侧收口。

## 本轮（glm-20261002T120930Z）已完成

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | **修后正式选参（r5 重扫）**：r4 判据（C6 物理化 + 1% 门槛）下 81 组网格全新实跑 162 runs（88s，`--workdir build/tuning_run_r5` 强制非缓存）——0 通过（62 C2 / 10 C3 / 9 C6：参考自身 + r3 的 8 组伪改进全被 1% 门槛机械拒绝）；全新实跑与 r3 产物重放逐行一致（确定性复证）；结论 = 维持参考配置、比较集评估权继续未消费；docs §11 如实记录 + 声明"本搜索空间已扫尽，再扫须先改 §8" | `eb308fe` | CI 37005963746 @`0f96dc8` **success** |

## 执行备注（下一棒注意）

- `tuning_driver.sh` 有**续跑语义**：workdir 里已有产物会跳过实跑（只复算判定）。
  本轮首跑就踩过——2 秒"完成"实为消费 r3 旧产物。强制重跑用 `--workdir` 指向新目录。
- docs-only 变更无需全量 drive_gates；`check-doc`（§8 一致性机检）必须复验。

## 历史摘要（详情见 git 历史 HANDOFF/STATE 版本）

- `glm-20261002T111615Z`：Nightly 36933923349 假红根因修复（safeStopResult 补 racing_status=4、fault_injection 孤儿清理、出口纯度预检），`2c9a4e6`，CI 37002942307 success。
- `qoder-20260929T231407Z`：#48/#50 关闭、调参腿 r3 不采纳、#51 关闭（`0c24c11`）。
- `relay-2026-09-29-A`：#17 ③④（afbfa65/8d94444/69aec5b）。**#17 不得用仿真证据关闭**。

## 活跃任务 / 待办（按优先级）

1. **#17/#19/#15/#16 软件侧收口**（缺标定/台架，不为关闭跳过硬验收；#17 尤其不许仿真证据关闭）。
2. #23/#24/#18/#25/#26/#27：需人工输入，无输入不臆造。
3. #37（needs-triage → 按规则跳过，需人工）。
4. 调参腿：**已收口**（r5，docs §11.3）——新扫描须先改 §8 声明（新 round + 预注册，先于候选）+ 新维度动机；比较集评估权保留给未来真实候选。

## 门禁状态（轮 1 收尾）

- 预检三连绿（selftest / check-doc / check，C6 豁免按设计）；docs-only 提交，代码零改动。
- 主干绿：CI 37002942307 @2c9a4e6 success（上一棒）。

## 环境事实（沿用，本轮复验成立）

- 多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`、`ROS_HOME=$PWD/build/.ros`；跑 scripts 前 source ROS **与** install/setup.bash。
- 清理只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空（用 comm 截断名）。
- 沙箱 /tmp 受限 → 临时产物写 `build/`。gh `run watch --interval` 不存在 → 轮询 `gh run view --json status`。
