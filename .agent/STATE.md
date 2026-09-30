# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`qoder-20260929T231407Z`｜时间范围：2026-09-29T23:14Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒台账：`relay-2026-09-29-A`（其叙述在 git 历史的 HANDOFF 版本里；本文件是事实账本）。
> 环境探测缓存见 `.agent/ENV.md`（首轮已生成，人工可编辑覆盖）。

## 当前活跃

- 轮 4 候选：**#51 [P2]**（C6 改进集混入定级指标⇒松绑伪改进；用 r3 产物重放验证，不重跑 runner）。

## 本轮（qoder-20260929T231407Z）已完成

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | **#48 [P2]** 调参准则可满足性预检落地：结构化声明 `config/mpc_tuning_declaration.json` + `tuning_precheck.py`（check/commands/evaluate/emit-doc/check-doc/selftest，15 例单测）+ `tuning_driver.sh` + drive_gates 硬门 `tuning-precheck` + 冻结文档 §8 自动区段；负样本 rc=1 指名 C9 / waived 后 rc=0；端到端 1 组合小网格通过并顺带抓到"产物口径混用"缺口（已加守卫） | `1e70583` | push 36646546721 **success** → **#48 已关闭**（评论 `#issuecomment-5901299200`） |
| 2 | **#50 [P2]** MPC 持续拒解运行期出口：RejectWatchdog 纯 std 核 + HuatControllerHealth.msg（/controller/mpc/health）+ 节点接线（拒解锁存后安全侧保持；新参数 reject_threshold/qp 四旋钮，默认=现值有单测钉住）+ 两臂冒烟 mpc_reject_smoke.sh + drive_gates 新硬门；6+1 单测双注册；偏差（独立脚本非 fault_injection 场景）已评论 | `466c8ff` | push 36651547804 **success** → **#50 已关闭**（评论 `#issuecomment-5901922909`） |
| 3 | **调参腿 r3 执行**（§7.6 同轮扫）：runner 加 acceptable_* 旋钮 + C6 strictly_better_some + 声明 81 组扫描（162 runs <5min）；**不采纳任何参数**——8 组"通过"全是指级松绑伪改进（rmse/fail/lap 平手到噪声位），§9 如实记录；缺口另立 **#51 [P2]**（本棒新建） | `746e9e6` | push CI 见下方（本棒轮 3） |

## 活跃任务 / 待办（按优先级）

1. **#51 [P2]**：C6 改进集收紧为物理指标（rmse/lap）+ 指级指标白名单机检 + r3 产物重放
   （evaluate 不需重跑 runner）；重声明必须在任何新候选运行之前。
2. **#17 / #19 / #15 / #16** 软件侧收口（缺标定/台架，不许为关闭跳过硬验收）。
3. #23/#24/#18/#25/#26/#27：需人工输入（真实 bag、ROS1 环境），无输入不臆造。
4. #37（needs-triage 标签 → 按规则跳过，需人工）。
5. r3 后的正式选参：等 #51 修复后重声明再扫（本轮无真实赢家；比较集评估权**未消费**）。

## 门禁状态（轮 3 收尾）

- 本机全量 drive_gates：**14/14 PASS、hard_fail=0、599s**（门名清单见 last.json：build/test-run/
  test-enforce/lint/cpp20/core-standalone/benchmark/tuning-precheck/headless-smoke/qos-contract/
  closed-loop/closed-loop-fault/fault-injection/mpc-reject-smoke）。
- 三条 v1 基线与第三臂基线：benchmark 门 PASS（未触碰 `benchmarks/baseline/`）。
- 主干绿：push run 36651547804 @466c8ff success；本棒累计 CI 样本：36646546721、36651547804 均 success。
- `colcon test` **483 tests / 0 fail**（#50 新增 6+1 例）；core_standalone 23 项全绿。

## 环境事实（沿用上棒实测，本轮验证仍成立）

- 本机 lyrical，UDP 组播被 VPN 挡 → 多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
- 断言型 ros2 查询走 daemon（lint_shell 第 2 段机检）。
- shellcheck：`build/sc_root/usr/bin`（apt 下载解包），PATH 前缀后跑 lint_shell。
- `colcon test --packages-ignore hello_world`；跑 scripts 前必须 source ROS **与 install/setup.bash**
  （本棒轮 2 又踩过一次：漏 source install → "Package not found" 假失败）。
- 沙箱 /tmp 只读 → 临时产物写 `build/`（本轮负样本 `build/neg_decl*.json`、e2e 在 `build/e2e_run/`，都不入库）。
- gh `run watch --interval` 参数不存在（本机版本）→ 轮询 `gh run view --json status` 等 CI。
