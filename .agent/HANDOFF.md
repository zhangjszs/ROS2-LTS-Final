# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`qoder-20260929T231407Z`（进行中，轮 3 收尾）。上一棒 `relay-2026-09-29-A` 台账在 git 历史。
先读 `.agent/STATE.md`（账本）与 `.agent/ENV.md`（命令缓存）。

## 一、本棒已收尾（轮 1–3）

- **#48**（`1e70583`，已关闭）：调参声明 + `tuning_precheck.py` + `tuning_driver.sh` +
  drive_gates 硬门 `tuning-precheck`。文档 §8 区段 = 声明的自动生成镜像，改声明必须
  `emit-doc --write`（否则门红）。
- **#50**（`466c8ff`，已关闭）：MPC 持续拒解出口（health 话题 + 安全侧保持 + 两臂冒烟
  `mpc_reject_smoke.sh` + 硬门）。QP 四设置也是节点参数（默认=现值，单测钉住）。
- **调参腿 r3**（`746e9e6`，docs §9）：runner 加 `--mpc-acceptable-primal/-dual`；
  C6 `strictly_better_some` + 真实语义豁免首跑；81 组扫完**不采纳任何参数**
  （8 个"通过"全是定级松绑伪改进，物理指标平手到噪声位）；比较集评估权未消费。
  缺口新建 **#51 [P2]**。
- 主干绿（CI 样本 36646546721 / 36651547804 / 36655463153 均 success）；
  drive_gates 14/14、hard_fail=0；483 tests/0 fail；工作树干净（产物在 build/ 不入库）。

## 二、下一棒的选题顺序

| 序 | 任务 | 为什么是它 | 注意 |
|---|---|---|---|
| 1 | **#51 [P2]** C6 改进集收紧（物理指标 only + 定级指标白名单机检） | open 且 label 优先级最高；r3 的直接产物 | **任何新候选运行之前**完成重声明；用 `build/tuning_run/` 产物直接 `evaluate` 重放验证 8 组应被拒（若产物目录已被清理，参考 `docs` §9.2 数字重扫一次网格即可，成本 <5min）；改后 `emit-doc --write` |
| 2 | 修后正式选参 + 比较集一次评估 | 评估权还在（未被 r3 消费） | 有真实赢家才改参数：qp_solver.hpp 默认值 → 连带 mpc 节点 declare 默认 + test_param_consistency 同步 + 全门 + 比较集一次（命令禁旋钮）；无赢家则 §追加记录即可 |
| 3 | #17/#19/#15/#16 软件侧收口 | 缺标定/台架 | 不为关闭跳过硬验收 |
| 4 | #23/#24/#18/#25/#26/#27 | 需人工输入 | 无输入不臆造 |
| — | #37 needs-triage | 规则跳过 | 需人工分诊 |

## 三、红线（全部仍然有效）

- 别"修 #47 的非单调判据"（不存在，§7 证伪）；别绕过预检跑网格；别看过结果后改判据
  （**收紧**也要走"下一轮重声明"流程并留理由——r3 的不采纳论证在 §9.2，照此办理）。
- 别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段；改共用层后必须
  `benchmark_regression.sh` rc=0。
- `QpSettings.warm_start` 死配置仍未处置。
- 别 force push / 自动 merge / main 上临时放宽门禁。

## 四、复现 r3（顺序照抄）

```bash
cd <repo>
export FASTDDS_BUILTIN_TRANSPORTS=SHM
source /opt/ros/lyrical/setup.bash && source install/setup.bash   # 两个都必须
python3 scripts/tuning_precheck.py selftest && python3 scripts/tuning_precheck.py check-doc
python3 scripts/tuning_precheck.py check          # 参考指标缓存 build/tuning_precheck/
bash scripts/tuning_driver.sh                     # 预检→81×2 扫描→report（<5min，产物 build/tuning_run/）
# #51 的重放验证：改完声明/C6 后直接
python3 scripts/tuning_precheck.py evaluate --workdir build/tuning_run   # 8 组应全拒
```
