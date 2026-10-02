# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`glm-20261002T120930Z`（2026-10-02，单轮收尾）。上一棒 `glm-20261002T111615Z`
修了 Nightly 假红（详见其 HANDOFF 与 STATE 历史摘要）。先读 `.agent/STATE.md` 与 `.agent/ENV.md`。

## 一、本棒做了什么：调参腿正式收口（r5 重扫）

在 r4 判据（#51 收紧后的 C6：改进集=物理指标 + 1% 实质性门槛）下，对 81 组网格
（eps_abs×eps_rel×max_iter×acceptable_primal，×2 场）做了 **162 runs 全新实跑**（88s）：

- **0 组通过**：62 拒于 C2（rmse）、10 拒于 C3（圈速）、9 拒于 C6（参考自身 c01 +
  r3 的 8 组伪改进，全部被 1% 门槛机械拒绝——rmse 相对改善 0.0163% ≪ 1%）。
- **确定性复证**：全新实跑的逐组判定与 r3 产物重放逐行一致（report diff 为空）。
- **结论**：维持参考配置，不冻结任何变更；比较集评估权继续未消费（不为消费而消费）。
- 记录：docs/MPC_TUNING_FREEZE.md **§11**（§1–§10 一字未改；check-doc 复验绿）。

### 关键坑（下一棒务必知道）

`tuning_driver.sh` 有**续跑语义**：workdir 已有产物 → 跳过实跑只复算判定。
本棒第一次跑 2 秒就"完成"，实际消费的是 09-30 的 r3 旧产物（build/tuning_run/）。
强制重跑必须 `--workdir` 指向新目录（本轮用 `build/tuning_run_r5/`）。

### 调参腿现状

本搜索空间在 r4 判据下**已扫尽**（docs §11.3）：同网格再扫是重复劳动；
新一轮扫描必须先改 §8 声明（新 round 号 + 预注册时间戳，先于任何候选运行）
并给出新维度动机（权重/时域/代价 shaping——判据语义尚未预留，需先过 §5 预检）。

## 二、下一棒的选题顺序

| 序 | 任务 | 注意 |
|---|---|---|
| 1 | #17/#19/#15/#16 软件侧收口 | 缺标定/台架的部分如实记录阻塞，不为关闭跳过硬验收；**#17 不许仿真证据关闭**（软件侧能收的子项先收） |
| 2 | #23/#24/#18/#25/#26/#27 | 需人工输入（真实 bag、ROS1 环境），无输入不臆造 |
| — | #37 needs-triage | 按规则跳过，需人工分诊 |

## 三、红线（全部仍然有效）

- 别看过结果后改判据；判据变更必须走"重声明（新 round + 预注册）→ 重放/重扫"流程。
- 别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段；改共用层必须 benchmark 门 rc=0。
- `QpSettings.warm_start` 死配置仍未处置。
- 别 force push / 自动 merge / main 上临时放宽门禁。
- 清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。
- docs-only 也要过 `check-doc`；scripts 变更过 lint_shell。

## 四、复现本轮（顺序照抄）

```bash
cd <repo>
source /opt/ros/lyrical/setup.bash && source install/setup.bash   # 两个都必须
python3 scripts/tuning_precheck.py selftest && python3 scripts/tuning_precheck.py check-doc
python3 scripts/tuning_precheck.py check
bash scripts/tuning_driver.sh --workdir build/tuning_run_r5b      # 新 workdir 强制实跑，~90s
python3 scripts/tuning_precheck.py evaluate --workdir build/tuning_run_r5b
# 预期：0/81 通过；9 组拒于 C6（含 c01 与 8 组伪改进）
```
