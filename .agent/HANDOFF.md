# HANDOFF.md — 给下一棒（relay 时间点交接）

当前棒：`glm-20261002T125512Z`（2026-10-02，单轮收尾）。上一棒 `glm-20261002T120930Z`
完成了调参腿 r5 收口。先读 `.agent/STATE.md` 与 `.agent/ENV.md`。

## 一、本棒做了什么：#15/#16/#17/#19 软件侧收口审计

逐项对照 main `361c478`，把四条路线图 issue 的**软件侧交付全部钉上证据**（commit 号、
单测名、门禁名，引用前先在库里核实存在），以证据映射评论挂到各 issue：

- **#15**（comment 5953555040）：共用 codec/标定单一来源（`9ab32bc`）、toMsg 唯一组装点
  （`1e3e1e6`）、非有限降级（`357ac9d`）、停车帧工厂（`2c9a4e6` 修复）——软件侧收口；
  剩余 = VCU 协议资料 + 实车标定（模板 BENCH_CALIBRATION_TEMPLATE.md 已备）。
- **#16**（…55810）：状态机/信任门/新鲜度租约单测 + fault_injection 10 场景 +
  closed_loop_fault + mpc_reject_smoke——软件侧验收矩阵闭环；剩余 = 硬件急停链路 +
  实车停车预算。
- **#17**（…5593）：基线版本化（`afbfa65`）、落终态（`8d94444`）、故障样本可识别
  （`69aec5b`/`2e25417`）、CI 硬门（`d2ef36c`）——③④兑现；剩余 = ①模型速度范围实测
  ②感知扰动量化（数据 → #18）。**不许仿真证据关闭**（issue 的 KPI 可信性主张本身要实车锚点）。
- **#19**（…7397）：A 阶段四场景基线 + 故障门全绿；B 阶段 constspeed 对照骨架 +
  调参/比较集分离机制（#45/#48）已在；剩余 = 完整公平对比（依赖 #17 扰动场景）。

**结论：四条全部保持 OPEN**，剩余阻塞无一可由软件侧推进（详见 STATE 待办 1）。
本棒零代码改动，只提交台账。

## 二、下一棒的选题顺序

| 序 | 任务 | 注意 |
|---|---|---|
| 1 | **等待输入类任务优先做"可软件推进的剩余子项"扫描**：四条 issue 的剩余项理论上都卡硬件/人工，但若 #18 拿到真实 bag、#23 确认外部驱动，立即按各自关闭条件接续 | 无输入不臆造 |
| 2 | #24（跨 ROS 版本语义差分回放）若 ROS1 环境可用则可软件侧先行 | 需人工确认 ROS1 环境 |
| 3 | #26/#25（showcase/博客） | 需人工决定展示范围 |
| — | #37 needs-triage | 按规则跳过 |

也就是说：**软件侧待办已扫尽**。下一棒若仍无人输入，建议做轻量维护轮（依赖更新评估、
文档一致性巡检、CI 样本积累），不要为"有事做"而开硬件依赖项的口子。

## 三、红线（全部仍然有效）

- 别用过仿真证据关闭 #17/#15/#16/#19——各 issue 评论里已写明关闭条件，照那执行。
- 别看过结果后改判据；判据变更走"重声明（新 round + 预注册）→ 重放/重扫"。
- 别动 `benchmarks/baseline/`；别往 `fsac.benchmark.kpi/v1` 加字段；改共用层必须 benchmark 门 rc=0。
- `QpSettings.warm_start` 死配置仍未处置。
- 别 force push / 自动 merge / main 上临时放宽门禁。
- 清理进程只按 PID/PGID；严禁 `pkill -f <节点名>`；`pgrep -x` 对 >15 字符进程名恒空。

## 四、沙箱环境备注（本轮新踩）

- 权限分类器可能长时间（30+ 分钟）不可用：写类 Bash（commit/push/gh write）被卡，
  只读 Bash 与文件 Write 正常。应对：先把要发的内容写成文件、能做的只读检查先做完，
  恢复后一次性执行写操作。评论文件在 `build/issue_comments/c{15,16,17,19}.md`（不入库）。

## 五、复现本轮审计

```bash
gh issue view 15 --json comments --jq '.comments[-1].body'   # 15/16/17/19 同理
# 证据核样（抽查评论里引用的锚点是否仍在）：
git log --oneline -1 9ab32bc 1e3e1e6 357ac9d 2c9a4e6 afbfa65 8d94444 69aec5b 2e25417 d2ef36c
ls src/safety/safety_monitor/test/ src/control/pure_pursuit/test/
ls benchmarks/baseline/ docs/BENCH_CALIBRATION_TEMPLATE.md
```
