# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`qoder-20260929T231407Z`（进行中，轮 2 收尾）。上一棒 `relay-2026-09-29-A` 的台账在 git 历史。
读我之前先读 `.agent/STATE.md`（账本）与 `.agent/ENV.md`（命令缓存）。

## 一、本棒已收尾（轮 1–2，均关闭）

- **#48**（`1e70583`，CI 36646546721 success）：调参声明 `config/mpc_tuning_declaration.json` +
  `scripts/tuning_precheck.py`（check/commands/evaluate/emit-doc/check-doc/selftest）+
  `scripts/tuning_driver.sh`（预检→扫网格→判定→报告）+ drive_gates 硬门 `tuning-precheck`。
  准则已重声明为"不劣于参考"型（先于新候选写定，事后改=check-doc 红）。
- **#50**（`466c8ff`，CI 36651547804 success）：MPC 持续拒解出口 `RejectWatchdog` +
  `/controller/mpc/health`（HuatControllerHealth.msg）+ 拒解锁存后安全侧保持 +
  两臂冒烟 `scripts/mpc_reject_smoke.sh` + drive_gates 硬门 `mpc-reject-smoke`。
  QP 四设置已暴露为**节点参数**（默认=qp_solver.hpp 现值，test_param_consistency 钉住）。
- 主干绿、工作树干净（产物全在 `build/` 不入库）、drive_gates 15/15、483 tests/0 fail。

## 二、下一棒的选题顺序

| 序 | 任务 | 为什么是它 | 注意 |
|---|---|---|---|
| 1 | **重做调参腿**（§7.6/§5） | #48 机制 + #50 参数入口都已就位，就差真跑 | ①给 benchmark_runner 加 `--mpc-acceptable-primal/-dual`（吃进 SetQpSettings 完整结构体，改动小；runner 旋钮要进 TUNING-KNOBS 标记）；②声明 grid 加 acceptable 维度 + `emit-doc --write`（否则 check-doc 红）；③`tuning_driver.sh`（先 --dry-run）扫完 → 结果**追加**为冻结文档新节；④如有赢家：单独 commit 改默认值 + 比较集一次评估（命令禁旋钮）；如 0 组通过：如实记"无可采纳"，**不得事后放宽准则** |
| 2 | #17/#19/#15/#16 软件侧收口 | 长期挂着，缺标定/台架 | 不为关闭跳过硬验收；软件侧做完 + 硬件阻塞列清单，issue 保持 OPEN |
| 3 | #23/#24/#18/#25/#26/#27 | 需人工输入 | 无输入不臆造数据源 |
| — | #37 needs-triage | 按接力规则跳过 | 需人工分诊 |

## 三、别做的事（红线）

- **别去"修 #47 的非单调判据"**：不存在（证伪链在 `docs/MPC_TUNING_FREEZE.md` §7）。
- **别绕过预检直接跑网格**；**别改声明却不 `emit-doc --write`**；别动 `benchmarks/baseline/`；
  别往 `fsac.benchmark.kpi/v1` 加字段（诊断走 `controller_diag/v1`）。
- **§7.4 教训别忘**：兜底带是承重墙，收紧它必须与 eps/max_iter **同轮**扫（这正是序 1 的前提）。
- 别 force push、别自动 merge、别在 main 堆"让 nightly 绿"的放宽。
- `QpSettings.warm_start` 死配置仍未处置：动它要么真实现要么删。

## 四、复现本棒验证（顺序照抄）

```bash
cd <repo>
export FASTDDS_BUILTIN_TRANSPORTS=SHM               # 本机 VPN 挡组播
source /opt/ros/lyrical/setup.bash && source install/setup.bash   # 两个都必须，少一个假失败
python3 scripts/tuning_precheck.py selftest && python3 scripts/tuning_precheck.py check-doc
bash scripts/tuning_driver.sh --dry-run             # 54 行网格枚举
bash scripts/mpc_reject_smoke.sh build/mpc_reject reject   # ~45s
bash scripts/mpc_reject_smoke.sh build/mpc_reject normal   # ~40s
bash scripts/drive_gates.sh                          # 15 门，约 10 分钟
```

新门禁的 Jazzy CI 侧证据：36651547804 的 Test/Enforce 步（新 gtest 编译+运行）；
mpc_reject_smoke 与 tuning-precheck 两门走 nightly（drive_gates），下一次 schedule 可核对。
