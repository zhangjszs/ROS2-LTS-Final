# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`glm-20261002T111615Z`（2026-10-02，单轮收尾）。上一棒 `qoder-20260929T231407Z`
在轮 3 收尾后提交 `0c24c11`（#51 修复，issue 已 CLOSED）随即中断，未写台账——本棒接管过期锁并补记。
先读 `.agent/STATE.md`（账本）与 `.agent/ENV.md`（命令缓存）。

## 一、本棒做了什么

**Nightly 36933923349（schedule，0c24c11）假红根因修复。** 现象：`mpc-reject-smoke` 门
`[reject]` 判据 `racing_status=4（实际 0）`，而同 commit 的 push CI 与前一次 Nightly 均绿。
根因是**两个独立缺陷叠加**：

1. **产品 bug**：`common_msgs/command_arbitration.h` 的 `safeStopResult()` 手写停车帧、
   漏设 `racing_status`（默认 0 上线）。违反 `makeSafeStopRaw` 注释里的契约
   （"停车状态字…禁止手写停车帧"）。已补 `=4` + `SafeFallbackIsDeterministicAndValid` 单测钉住。
2. **门禁泄漏**：`fault_injection_smoke.sh` 场景 10 重启的仲裁器由 python harness `Popen` 拉起，
   harness 退出后成孤儿（EXIT 清理只覆盖 shell 端 PIDS），存活到 mpc-reject-smoke 并向
   `/vehicle_command` 发 status=0 的 safeStop 帧，恰好污染 `last_cmd_after_rejecting` 判据
   （CI 日志 job 末尾 "Terminate orphan process: command_arbiter_node" 是铁证）。
   已改 `start_new_session=True` + 退出时 `killpg` 整组清 + pidfile 供 shell 端兜底。
3. **防御**：`mpc_reject_smoke.sh` 采样前新增纯度预检——`/vehicle_command` 发布者数必须=1，
   否则列节点表判红，把跨门污染从"偶发假红"变成"确定性、指名道姓的失败"。

**验证**：safety_monitor 61 测全绿；两臂冒烟 PASS（racing_status=4、纯度预检绿）；
fault_injection 10/10 且按 comm 截断名复查无 `command_arbiter` 残留、pidfile 已清；
全量 drive_gates **14/14 PASS、hard_fail=0、505s**。`benchmarks/baseline/` 未触碰。

## 二、下一棒的选题顺序

| 序 | 任务 | 注意 |
|---|---|---|
| 1 | **修后正式选参**：重扫 81 组网格（`bash scripts/tuning_driver.sh`，<5min） | 收紧后的 C6 下 8 组伪改进应全拒；有真实赢家才动参数（qp_solver.hpp 默认 + 节点 declare + test_param_consistency 同步 + 全门 + 比较集一次，命令禁旋钮）；无赢家则 docs §9 追加记录。比较集评估权未消费 |
| 2 | #17/#19/#15/#16 软件侧收口 | 缺标定/台架，不为关闭跳过硬验收；#17 尤其不许仿真证据关闭 |
| 3 | #23/#24/#18/#25/#26/#27 | 需人工输入（真实 bag、ROS1 环境），无输入不臆造 |
| — | #37 needs-triage | 按规则跳过，需人工分诊 |

另：#51 的关闭缺证据评论（上一棒直接 commit-close）。若在意留痕，可补一条指向 `0c24c11`
与本棒 Nightly 根因的评论；非必做。

## 三、红线（全部仍然有效）

- 别看过结果后改判据；**收紧**也要走"下一轮重声明"流程并留理由（§9.2 先例）。
- 别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段；改共用层后必须
  benchmark 门 rc=0（本轮已照办）。
- `QpSettings.warm_start` 死配置仍未处置。
- 别 force push / 自动 merge / main 上临时放宽门禁。
- 清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`。`pgrep -x` 对 >15 字符进程名恒空，
  复查残留要用 comm 截断名（如 `pgrep -x command_arbiter`）。

## 四、复现本棒的验证（顺序照抄）

```bash
cd <repo>
export FASTDDS_BUILTIN_TRANSPORTS=SHM
source /opt/ros/lyrical/setup.bash && source install/setup.bash   # 两个都必须
colcon test --packages-select safety_monitor && colcon test-result --verbose \
  --test-result-base build/safety_monitor                          # 61/61
bash scripts/mpc_reject_smoke.sh build/mpc_reject_check reject     # racing_status=4、纯度=1
bash scripts/mpc_reject_smoke.sh build/mpc_reject_check normal
bash scripts/fault_injection_smoke.sh build/fault_injection_check  # 10/10；收尾 pgrep -x command_arbiter 应空
bash scripts/drive_gates.sh                                        # 14/14，~9min
```
