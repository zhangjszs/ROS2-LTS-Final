# HANDOFF.md — 给下一棒（relay 时间点交接）

上一棒：`relay-2026-09-29-A`（其完整台账在 git 历史的 STATE/HANDOFF 版本与 `docs/MPC_TUNING_FREEZE.md` §7）。
当前棒：`qoder-20260929T231407Z`（进行中）。读我之前先读 `.agent/STATE.md`（账本）与 `.agent/ENV.md`（命令缓存）。

## 一、本棒已收尾（轮 1）

- **#48 已修复并关闭**（commit `1e70583`，CI push run 36646546721 success）：
  调参现在有结构化声明 `config/mpc_tuning_declaration.json` + 唯一入口
  `scripts/tuning_precheck.py`（check/commands/evaluate/emit-doc/check-doc/selftest）+
  驱动 `scripts/tuning_driver.sh`；开跑前强制"参考配置可满足性预检"，不成立且未
  `waived` 即拒绝并指名；文档 §8 AUTO-DECLARATION 区段与声明漂移即报错（drive_gates
  新硬门 `tuning-precheck`，nightly 覆盖）。准则已按 §5 重声明为"不劣于参考"型 +
  新增兜底接受率准则 C5；声明里的 `known_gaps` 记着 `acceptable_*` 还不是旋钮。
- 主干绿；工作树干净（未提交产物全在 `build/`，不入库）；`.agent/LOCK` 结束时删除。

## 二、下一棒的选题顺序

| 序 | 任务 | 为什么是它 | 注意 |
|---|---|---|---|
| 1 | **#50 [P2]** MPC 持续拒解运行期出口 | 上一棒 HANDOFF 顺序 2；§7.4 暴露的真实缺陷 | **只加出口**（计数/状态/冒烟场景），不改判据/参数；触发场景可把 eps 压到 1e-14 |
| 2 | **重做调参腿**（§7.6/§5：新准则下扫 27 组） | #48 机制已就位，这就是它的第一个用户 | 用 `bash scripts/tuning_driver.sh`（先 --dry-run）；别手写一次性脚本；§7.6 要求兜底带同轮扫——先给 runner 加 `--mpc-acceptable-primal` 类旋钮（小改动，另立 commit）或按 known_gaps 明示只扫二维；比较集命令永远不得含旋钮；跑完把结果**追加**为冻结文档新一节（不覆盖） |
| 3 | #17/#19/#15/#16 软件侧收口 | 长期挂着，缺标定/台架 | 不为关闭跳过硬验收；软件侧做完 + 硬件阻塞列清单，issue 保持 OPEN |
| 4 | #23/#24/#18/#25/#26/#27 | 需人工输入（真实 bag、ROS1 环境） | 无输入不臆造数据源 |

## 三、别做的事（红线，沿用上棒 + 本轮新增）

- **别去"修 #47 的非单调判据"**：不存在，实测证伪链在 `docs/MPC_TUNING_FREEZE.md` §7。
- **别动 `benchmarks/baseline/`**；改共用层后 `benchmark_regression.sh` 必须 rc=0。
- **别往 `fsac.benchmark.kpi/v1` 加字段**；诊断字段进 `controller_diag/v1`。
- **别绕过预检直接跑网格**——`tuning_driver.sh` 故意先跑 check；一次性脚本改判据正是 #48 要堵的洞。
- 别在 main 上做"让 nightly 变绿"的临时放宽；别 force push；别自动 merge PR。
- `QpSettings.warm_start` 仍是死配置（`qp_solver.hpp:20`）；动它要么真实现要么删。
- 新增准则/改声明后必须 `emit-doc --write` 再提交，否则 `check-doc` 门当场红（这是设计）。

## 四、复现本棒验证（顺序照抄）

```bash
cd <repo>
export FASTDDS_BUILTIN_TRANSPORTS=SHM               # 本机 VPN 挡组播
source /opt/ros/lyrical/setup.bash && source install/setup.bash
python3 scripts/tuning_precheck.py selftest         # 15 例，秒级
python3 scripts/tuning_precheck.py check-doc
python3 scripts/tuning_precheck.py check            # 实跑两场参考（<1s），产物在 build/tuning_precheck/
bash scripts/tuning_driver.sh --dry-run             # 预检 + 枚举 54 行网格（不实跑）
# 负样本重放：把旧准则①(at_least finished, 无 waived)追加进声明副本 → driver rc=1 指名；加 waived → rc=0
PATH=$PWD/build/sc_root/usr/bin:$PATH bash scripts/lint_shell.sh
bash scripts/drive_gates.sh                          # 现为 13 门，约 9–10 分钟
```
