# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`qoder-20260929T231407Z`｜时间范围：2026-09-29T23:14Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒台账：`relay-2026-09-29-A`（其叙述在 git 历史的 HANDOFF 版本里；本文件是事实账本）。
> 环境探测缓存见 `.agent/ENV.md`（首轮已生成，人工可编辑覆盖）。

## 当前活跃

- 无挂起写入。轮 3 候选：**重做调参腿**（§7.6：先给 runner 把 acceptable_* 暴露为旋钮，
  声明加维度，再用 tuning_driver.sh 扫全网格）；若产生赢家才涉及参数变更 + 比较集一次评估。

## 本轮（qoder-20260929T231407Z）已完成

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | **#48 [P2]** 调参准则可满足性预检落地：结构化声明 `config/mpc_tuning_declaration.json` + `tuning_precheck.py`（check/commands/evaluate/emit-doc/check-doc/selftest，15 例单测）+ `tuning_driver.sh` + drive_gates 硬门 `tuning-precheck` + 冻结文档 §8 自动区段；负样本 rc=1 指名 C9 / waived 后 rc=0；端到端 1 组合小网格通过并顺带抓到"产物口径混用"缺口（已加守卫） | `1e70583` | push 36646546721 **success** → **#48 已关闭**（评论 `#issuecomment-5901299200`） |
| 2 | **#50 [P2]** MPC 持续拒解运行期出口：RejectWatchdog 纯 std 核 + HuatControllerHealth.msg（/controller/mpc/health）+ 节点接线（拒解锁存后安全侧保持；新参数 reject_threshold/qp 四旋钮，默认=现值有单测钉住）+ 两臂冒烟 mpc_reject_smoke.sh + drive_gates 新硬门；6+1 单测双注册；偏差（独立脚本非 fault_injection 场景）已评论 | `466c8ff` | push 36651547804 **success** → **#50 已关闭**（评论 `#issuecomment-5901922909`） |

## 活跃任务 / 待办（按优先级）

1. **重做调参腿**（§7.6 + §5 第 2/3 步；声明 known_gaps 已记账）：先给 benchmark_runner 暴露
   `--mpc-acceptable-primal/-dual` 旋钮（SetQpSettings 吃完整 QpSettings，改动小），声明加维度，
   `tuning_driver.sh` 扫网格 → 结果追加为冻结文档新节 → 如有赢家再单独 commit 改参数 + 比较集一次。
2. **#17 / #19 / #15 / #16** 软件侧收口（缺标定/台架，不许为关闭跳过硬验收）。
3. #23/#24/#18/#25/#26/#27：需人工输入（真实 bag、ROS1 环境），无输入不臆造。
4. #37（needs-triage 标签 → 按规则跳过，需人工）。

## 门禁状态（轮 2 收尾）

- 本机全量 drive_gates：**15/15 PASS、hard_fail=0、599s**（新增 tuning-precheck 与
  mpc-reject-smoke 两门；后者两臂各 ~40s）。
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
