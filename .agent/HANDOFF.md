# HANDOFF.md — 给下一棒（relay 时间点交接）

上一棒：`relay-2026-09-29-A`，止于 2026-09-29T16:35Z，main = `d0ecb52` + 本状态文件的 commit。
读我之前先读 `.agent/STATE.md`（同一轮的状态与实测数字）与 `.omc/unattended/progress.md`（更早的 R5 台账）。

## 一、本棒收尾状态（无挂起项）

- 主干绿：push run **36591703745** @ `d0ecb52` 与 **36594654367** @ `9028aab` 全 22 步 success；
  手动 nightly **36591912422** @ `d0ecb52` 也 success（Jazzy 上 `drive_gates` 12/12 PASS、
  hard_fail=0、562s —— #49 在云端也确认修好了）。
- issue：**#49 / #47 / #41 均已关闭**（#49 与 #47 都是本棒新建后修复并关闭的；#47 标题已改为
  `[已证伪] …`，证据/关闭评论在 `#issuecomment-5893504355` 与 `#issuecomment-5893854199`）；
  新建且**保持 OPEN** 的是 **#50 [P2]**。
- **纠正一条过时认知**：三条容差型集成门禁**早已转正为硬阻断**
  （`d2ef36c feat(#17): 三条容差型集成门禁由观察期转正为 CI 硬阻断`，`ci.yml` 里现在没有
  `continue-on-error`）。所以不存在“凑够连续绿就摘掉”的待办；#49 的性质是硬门假失败 = 主干红。
  本棒的改动没有动任何门预算/阈值/步骤数。
- 工作树干净；未提交产物全在 `build/`（探针与日志，不入库）。今晚 18:00 UTC 还有一次 cron nightly，
  可作为下一棒的第一个核对项。
- round-4 额外交付：把手工契约“断言型查询不得用 `--no-daemon`”做成机检（`lint_shell.sh` 第 2 段
  必查，只匹配真实执行行；负样本 rc=1；全门 drive_gates 12/12 PASS、602s）。从此 #49 同类复发
  会在 lint 门当场变红，而不是等一次 CI push 才发现。

## 二、下一棒的选题顺序（按当前仓库真实状态排的）

| 序 | 任务 | 为什么是它 | 注意 |
|---|---|---|---|
| 1 | **#48 [P2]** 调参准则可满足性预检 + 按新准则**重做调参腿** | §7 已给出输入：`eps=1e-4 / max_iter=50 / band=0.25` 三个数互相差 3–4 个数量级，必须**同轮**扫；本轮新增的 `converged/accepted_approx` 计数就是判据数据源 | 别再单独收紧兜底带（§7.4 实测否掉了）；准则要先写死再跑；比较集只用一次 |
| 2 | **#50 [P2]** MPC 持续拒解的运行期出口（计数 + 状态 + 冒烟场景） | 由 §7.4 暴露，我按"每轮最多 1 个主动新 issue"没顺手做 | 只加出口，**不改判据/参数**；场景可以用人为把 eps 压到 1e-14 来触发 |
| 3 | **#17 / #19 / #15 / #16** 的软件侧收口 | 长期挂着，缺的都是**标定/台架**，不是代码 | 不许为了关闭而跳过硬验收：软件侧做完 + 硬件阻塞项列清单，issue 保持 OPEN |
| 4 | #23/#24/#18/#25/#26/#27（迁移清单/差分回放/rosbag/博客/展示） | 大特性，需要人给输入（真实 bag、ROS1 环境） | 不要在没有输入时臆造数据源 |

## 三、别做的事（我这轮踩到或验证过的红线）

- **别去"修 #47 的非单调判据"**：不存在。所有相关测量、方法、复现命令都在 `docs/MPC_TUNING_FREEZE.md` §7。
- **别动 `benchmarks/baseline/` 与三条 v1 基线**；改共用层后必须验证 `benchmark_regression.sh` rc=0
  （PP 默认路径逐字节一致）——本轮 MPC-only 改动已经验证过一遍，是零漂移。
- **别往 `fsac.benchmark.kpi/v1` 加字段**（它是基线口径锚点）；诊断字段进 `controller_diag/v1`（本轮就是这么做的）。
- 别在 `main` 上堆"让 nightly 变绿"的临时放宽；别 `--force-push`；别自动 merge PR。
- `QpSettings.warm_start` 是**声明了但没人读**的死配置（`qp_solver.hpp:20`）；动它要么真实现要么删，
  别留着当"看起来可配"。本轮没动它（超出 #47 范围）。

## 四、复现我这轮的验证（顺序照抄即可）

```bash
cd <repo>
export FASTDDS_BUILTIN_TRANSPORTS=SHM               # 本机 VPN 挡组播
source /opt/ros/lyrical/setup.bash && source install/setup.bash
bash scripts/lint_cpp.sh && bash scripts/lint_shell.sh
MAKEFLAGS=-j4 colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=ON \
  -DCMAKE_LINKER_TYPE=MOLD -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
colcon test --packages-ignore hello_world && colcon test-result
bash scripts/core_standalone_check.sh
bash scripts/benchmark_regression.sh              # 必须 source 过 workspace，否则假失败
bash scripts/drive_gates.sh                        # 12 门，约 9–10 分钟
# #47 的分解计数（新增字段就是这么读的）
./build/track_benchmark/benchmark_runner --track trackdrive --controller mpc --corridor-scale 0.9 \
  --require-laps 1 --rmse-max 0.6 --timeout 400 --mpc-max-iter 50 --diag-out build/x.diag.json 2>&1 | grep 'diag: solves'
```

实例级单调性重放没有 CLI 入口（我这轮用的是一次性探针，未入库，思路写在 §7.1）：
拿一个 `max_iter` 的闭环轨迹记录 `(x,y,theta,v,prev_steer,prev_accel)`，再逐实例换 `max_iter`
冷解比对接受位图——`MpcModel::Step` 每次先 `Reset()`，所以逐实例重放是完全确定性的。
