# STATE.md — 时间流接力状态（当前活跃 Agent）

> agent-id：`qoder-20260929T231407Z`｜时间范围：2026-09-29T23:14Z –（进行中）｜环境：本机 lyrical（CI 是 Jazzy）
> 上一棒台账：`relay-2026-09-29-A`（其叙述在 git 历史的 HANDOFF 版本里；本文件是事实账本）。
> 环境探测缓存见 `.agent/ENV.md`（首轮已生成，人工可编辑覆盖）。

## 当前活跃

- 无挂起写入。轮 2 候选：**#50 [P2]**（MPC 持续拒解运行期不可见：计数 + 状态出口 + 冒烟场景），
  按上一棒 HANDOFF 顺序第 2 位（#48 已在轮 1 完成并关闭）。

## 本轮（qoder-20260929T231407Z）已完成

| 轮 | 任务 | commit / issue | CI |
|---|---|---|---|
| 1 | **#48 [P2]** 调参准则可满足性预检落地：结构化声明 `config/mpc_tuning_declaration.json` + `tuning_precheck.py`（check/commands/evaluate/emit-doc/check-doc/selftest，15 例单测）+ `tuning_driver.sh` + drive_gates 硬门 `tuning-precheck` + 冻结文档 §8 自动区段；负样本 rc=1 指名 C9 / waived 后 rc=0；端到端 1 组合小网格通过并顺带抓到"产物口径混用"缺口（已加守卫） | `1e70583` | push 36646546721 **success** → **#48 已关闭**（评论 `#issuecomment-5901299200`） |

## 活跃任务 / 待办（按优先级）

1. **#50 [P2]** MPC 持续拒解运行期出口（只加出口，不改判据/参数；可用人为 eps→1e-14 触发场景）。
2. **重做调参腿**（冻结文档 §7.6 + §5 第 2/3 步）：用刚落地的 `tuning_driver.sh` 在新准则下扫 27 组；
   §7.6 还要求兜底带同轮扫——需先把 `acceptable_*` 暴露为 runner 旋钮（声明 known_gaps 已记账）。
3. **#17 / #19 / #15 / #16** 软件侧收口（缺标定/台架，不许为关闭跳过硬验收）。
4. #23/#24/#18/#25/#26/#27：需人工输入（真实 bag、ROS1 环境），无输入不臆造。

## 门禁状态（轮 1 收尾）

- 本机全量 drive_gates：**13/13 PASS、hard_fail=0、560s**（新增 tuning-precheck 门，链路：
  selftest && check-doc && check；check 实跑两场参考仅 ~0.7s）。
- `colcon test` 475/0 fail（本轮无 C++ 改动，build/test 由 drive_gates 覆盖通过）。
- 三条 v1 基线与第三臂基线：benchmark 门 PASS（未触碰 `benchmarks/baseline/`）。
- 主干绿：push run 36646546721 @1e70583 success；此前 nightly 36635488477（schedule）success。

## 环境事实（沿用上棒实测，本轮验证仍成立）

- 本机 lyrical，UDP 组播被 VPN 挡 → 多节点脚本前 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。
- 断言型 ros2 查询走 daemon（lint_shell 第 2 段机检）。
- shellcheck：`build/sc_root/usr/bin`（apt 下载解包），PATH 前缀后跑 lint_shell。
- `colcon build` 必须重列 mold/ccache；`colcon test --packages-ignore hello_world`。
- 沙箱 /tmp 只读 → 临时产物写 `build/`（本轮负样本 `build/neg_decl*.json`、e2e 在 `build/e2e_run/`，都不入库）。
- gh `run watch --interval` 参数不存在（本机版本）→ 轮询 `gh run view --json status` 等 CI。
