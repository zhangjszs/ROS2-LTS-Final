# TASKS.md — Issue 本地镜像（权威：GitHub Issue；本文件由 Planner 每轮整体重写）

> 维护者：Planning Agent｜最后更新：2026-10-21（`plan-20261021` 轮：**验收关闭 #78/#79/#80/#81/#82 五单 ⇒ Milestone S2a「直线加速事件闭环」达成**；主线暂歇，今日队列切至 P3 积压 **#83/#84/#85**；共 **28 open**）
> 规则：本文件只读镜像，Executor 改状态时先改 Issue 再同步对应行；冲突以 Issue 为准。
> 日报：`.agent/DIARY/2026-10-20.md`（三会话 + 补记）已由上一执行轮产出，本轮验收已逐条引用并独立复核。
> 🆕 **本轮变化**：昨日五单全部通过验收并关闭（独立复核证据见 PLAN §三）。Planner 裁 **D-018**（`acceleration_v1` 口径变更对 D-003 的传导：今日不重声明调参轮次，转为未来 round 预注册义务）；新建 **pending D-019**（S2a 后方向，默认 B）；把 daemon 预热回填 5 脚本并入既有 **#69**（comment `6296201990`）。
> ✅ **验收性质说明**：五单报告均**属实**，无「报告失实」项；且 #82 会话 3 主动更正了会话 2 一份**失实的 `blocked` 报告**（误判「本机 DDS 不可用」→ 真因 ros2cli daemon 未显式启动），本轮以门禁留痕独立验证更正为真（closed-loop/fault/closed-loop-fault 三门均 PASS）。

## 今日队列（2026-10-21，给 Executor 的建议顺序）

> 主线 S2a 已闭环，今日为 P3 积压清理。**⚠ #83 与 #84 同处 `src/planning/straight_line_planner/include/line_fit_core.hpp` 与 `test/test_line_fit_core.cpp` 文件族**：该文件已含 #78 落地的 `hough_min_inlier_ratio > 1.0` 守卫，改动时**不得覆盖**；两单**必须串行**（先其一合入、`git pull --rebase` 后再做其二），不可并行开分支。#85 为纯 docs，风险最低，可任意时机。

1. **#83** `hough_threshold` 死参数（P3）：节点/YAML/`LineDetectorConfig` 三处声明但 `HoughFitCore` 从未读取（Planner 实测 0 vs 1000 输出逐位相同）。**推荐方案 A**（从三处删除，使暴露面与实现一致）；若选 B（接线 `adaptive_thresh=std::max(hough_threshold, dynamic_min)` 且默认设 3）须在报告说明。**硬约束：默认配置下 `HoughFitCore` 输出逐字不变**（用 #62 黄金样本对比）。
2. **#84** `temporal_filter_*` / `kCenterMargin` 的 NaN/越界静默失效（P3）：实测 alpha=NaN→NaN 且 `valid=true`、jump=NaN→跳变抑制静默关闭、center_margin=NaN→全部锥桶丢弃。在核心边界加参数清洗/校验（回退默认+WARN 或 拒绝帧，二选一），**不得以 `valid=true` 返回含 NaN 的线**、不得静默清空锥桶；补 NaN/±inf/负值/alpha>1 各 ≥1 例。
3. **#85** `docs/MIGRATION_BASELINE.md` 回填 D-014/15/16 决策引用 + 新增/补 `docs/README.md` 文档索引挂入 5 篇文档入口（P3）：仅 `docs/`；保持五值状态枚举、不改结论边界、不替代实车证据（D-004）。

> 队列 = 3 项（2–5 区间内）。做完即 ready-for-agent **见底**——后续方向见 pending D-019（默认 B：完成 P3 后进入 D-009 维持模式）。

## ready（可执行，3）

| Issue | 标题 | 优先级 | 依赖 / 要点 |
| --- | --- | --- | --- |
| #83 | `hough_threshold` 死参数（节点/YAML 暴露但算法未使用） | P3 | 无；实测 0 与 1000 输出逐位相同；方案 A 删除 / B 接线；**勿覆盖 #78 守卫** |
| #84 | temporal_filter_* 与 kCenterMargin 的 NaN/越界静默失效 | P3 | 无；实测 alpha=NaN 输出 NaN 且 valid=true；**与 #83 同文件族，须串行** |
| #85 | MIGRATION_BASELINE 回填 D-014/15/16 并挂入文档入口 | P3 | 无；仅 docs/；不改变结论边界 |

## in-review（待验收，0）

无。昨日五单 #81/#80/#78/#79/#82 已于本轮验收关闭（见「已关闭」）。

## in-progress（0）

无。

## 仓库外阻塞（7，均为外部输入缺失，非代码可推进）

| Issue | 标题 | 阻塞原因（D-013 分类） | 本轮复核 |
| --- | --- | --- | --- |
| #15 | 共用执行器适配层与车辆标定配置（P1） | **资料缺失**：VCU 协议文档、台架标定实值（D-016 已确认非外部代码缺失） | `updatedAt` 10-19；性质已澄清，仍不可执行 |
| #16 | 统一任务与安全状态、最终指令仲裁（P1） | **资料缺失**：硬件急停链路、实车停车预算 | `updatedAt` 10-05，无新评论 |
| #17 | 可复现仿真场景、可信 KPI 与自动回归（P1） | **资料缺失**：模型速度范围实测、感知扰动量化 | 同上 |
| #19 | Pure Pursuit 低速基线与 MPC 公平对比（P1） | **资料缺失**：依赖 #17 扰动场景 | 同上 |
| #18 | rosbag 数据集与感知质量评估（P2） | **资料缺失**：真实 bag（亦是 D-015 定位后端与 S2b 八字的前置） | 同上 |
| #24 | 跨 ROS 版本语义差分回放与故障回归（P1） | **运行时缺失**：验收第 2 条需 ROS1 运行环境（本机无 Noetic、无 docker）；第 1 条已由 #79 完成 | 第 1 条切片已交付关闭；本体仍卡运行时 |
| #37 | 翻转 use_arbiter 默认（P3，needs-triage + blocked） | **资料缺失**：需 #15 标定完成 | `updatedAt` 10-09，无新评论 |

> D-013 复查：本轮无外部输入到位（各单 `updatedAt` 未变），维持 blocked。剩余每轮零成本复查即可，不主动扫描（D-009）。

## needs-info / 待条件（3）

| Issue | 标题 | 等待 |
| --- | --- | --- |
| #25 | 技术博客系列（P2） | #17①② / #18 证据 |
| #26 | 完整项目展示（P2，父跟踪） | 同上 |
| #27 | 跨仓库路线总入口（P2） | 同上 |

## ready-for-human（14，用户跟进）

#63 CI 路径过滤与并发取消（P2）· #64 `check_cpp20.sh` 参数统一（P2）· #65 closed-loop-fault 六臂并行（P2）· #66 CI job 三段拆分（P2）· #67 `affected_gates.sh`（P3）· #68 ccache 键头文件通配（P3）· **#69 DDS 环境性失败区分（P2，本轮已并入「daemon 预热回填其余 5 脚本」范围，comment `6296201990`）**· #70 门禁时长预算监控（P3）· #71 core_standalone 覆盖率基线（P3）· #72 Prepare ROS 2 步骤超时（P3）· #73 评分核测试双轨去重（P3）· #74 本地快速反馈文档（P3）· #75 测试编写规范入库（P3）· #76 伞形：12 个仅 colcon 轨测试下沉（P3）

## 路线图伞形（1，长期跟踪，无优先级标签系有意豁免）

| Issue | 标题 |
| --- | --- |
| #20 | ROS 2 车队开发路线：运行基线 → 可信仿真 → 低速闭环 → 性能优化 |

## 已关闭（近期备查）

- **#82**（2026-10-21 验收关闭；`343ecc3`+`40673b0`/merge `e5f9998`；comment `6296139845`）：ROS 闭环完赛门禁 accel-finish，正样本 rc=0 + `ENABLE_FINISH_EVENT=0` 负样本 rc=1 + `MAX_V=20` 对照臂，完整 drive_gates 12 门全 PASS。含对旧 `blocked` 误判的更正。CI `39441369149`/`39453577195` success。
- **#80**（2026-10-21 验收关闭；`3870249`/merge `126cc83`；comment `6296132845`）：直线加速完赛判据（finish_judge.hpp 纯 std 核 + 门禁第 9 节 + 可证伪）；acceleration 基线重录 stopped false→true（口径变更 → D-018）。CI `39416500750` success。
- **#81**（2026-10-21 验收关闭；`1aae347`/merge `0f48b3a`；comment `6296130011`）：赛道元数据 finish_x/runout 进 TrackDefinition+tracks.json；CSV/baseline 逐字节不变。
- **#79**（2026-10-21 验收关闭；`0654d0e`/merge `ea5e19b`；comment `6296137702`）：GeodeticToEnu 下沉 geodetic_core.hpp（零 ROS 依赖，位模式差分 TOTAL_MISMATCH=0）。CI `39423649207` success。
- **#78**（2026-10-21 验收关闭；`f56ff36`/merge `a71b548`；comment `6296135114`）：hough_min_inlier_ratio 增 [0,1] 上界守卫消除 float→int UB（先红后绿）。CI `39422351603` success。
- 更早：#77 / #62 / #23（2026-10-20）；#59 / #60 / #61（2026-10-12）；#55 / #56 / #57 / #58。

## 环境与流程注意（本轮实测/复核）

- 基线：`main` = `origin/main`（**0/0 同步**，无分叉）。本轮为纯验收轮，**未产生任何代码提交**，只读验证。工作区仅外部 `M AGENTS.md`（非本体系产生，不碰、不代提交；提交 `.agent/` 只 `git add` 指定文件，严禁 `-a`/`-A`）。
- **独立复核证据（本轮 Planner 第一方，非仅采信报告）**：
  - `bash scripts/core_standalone_check.sh` → EXIT=0，**32/32 通过、ASan+UBSan 干净**，含 `test_track_generator`(#81)/`test_finish_judge`(#80)/`test_line_fit_core`(#78)/`test_geodetic_core`(#79)。
  - `build/drive_gates/last.json` 直读 → 完整 drive_gates **12 门全 PASS、hard_fail=0、440s**（含 accel-finish 64s、closed-loop 11s、closed-loop-fault 225s、fault-injection 45s）。
  - `git show 3870249` 亲验 acceleration 基线口径变更；`git log` 亲验 skidpad/trackdrive 基线最近改动为旧提交 `afbfa65`(#17)，本轮三单未触碰。
- **本机 runtime 门禁可用（关键事实，已推翻上一轮误判）**：真因是 ros2cli daemon 需显式启动（`stop`+`start`+`sleep 3`），非「DDS 发现不可用」。跨进程 DDS 诊断的 pub/sub 必须在**同一次执行**内（分两次沙箱调用永远不通）。
- **远程遗留分支**：`origin/agent/issue-81-track-metadata`（#81 已合入 `0f48b3a` 却未删，违契约 1.9）。本轮**未擅自删除**（属前序 Executor 遗留分支；删远程分支非验收轮产出范围）。已亲验该分支为 main 祖先（tip `1aae347`），**删除安全**；**请下一执行轮清理**，PLAN「给 Executor 的指令」已提示。
- 支持环境（D-017）：CI = ROS 2 **Jazzy**，本机 = **lyrical**；验证入口唯一 `bash scripts/drive_gates.sh`（`QUICK=1` 快速轨）。**ROS1 运行时不在支持范围**（对照基线 `FSAC/2025`）。
- `gh` 正常（账号 `zhangjszs`）；`gh` **无** `milestone` 子命令；`gh issue list --json` 不接受 `commentCount`。
- CI/Nightly（本轮核实 2026-10-21T00:57Z）：origin/main `761817e` CI `39456608722` success；10-20 执行轮全部代码提交 CI 亦 success（#78/#79/#80/#81/#82）。Nightly 最新 `39331301871` @ `b27fc64`（10-19 调度）success；10-20 夜间任务约 20:00Z 调度，下一轮入账。
