# docs/ 文档索引

本目录收录项目的工程文档。新增文档时请在本索引登记一行；文档间的权威关系（契约、决策、冻结协议）以下列各自正文为准。

## 核心文档

| 文档 | 内容 |
|---|---|
| [INTERFACE_CONTRACT.md](INTERFACE_CONTRACT.md) | 契约话题单一权威：话题名 / QoS / 坐标系 / 单位 / 速度语义（与 `common_msgs::contract` 配套，改动须同步 `scripts/qos_contract_check.sh`） |
| [CORE_ROS_BOUNDARY_ADR.md](CORE_ROS_BOUNDARY_ADR.md) | Core/ROS 边界架构决策：算法逻辑落纯 std Core（header-only、C++20），ROS 留在适配层；双轨测试（colcon 轨 + `tests/core_standalone/`） |
| [MIGRATION_BASELINE.md](MIGRATION_BASELINE.md) | ROS1(LTS) → ROS2 功能与接口迁移覆盖清单（固定提交源码对照基线；受影响条目引用 `.agent/DECISIONS.md` 的 D-014/D-015/D-016 处置决策） |
| [MPC_TUNING_FREEZE.md](MPC_TUNING_FREEZE.md) | 调参冻结协议：调参集选参 → 冻结 → 比较集单次评估；判据变更须重声明 + 预注册（唯一入口 `scripts/tuning_driver.sh`） |
| [TEST_PERF.md](TEST_PERF.md) | 测试与门禁性能台账：各门禁耗时、优化记录与实测数据 |

## 其他工程文档

| 文档 | 内容 |
|---|---|
| [BUILD_OPTIMIZATION.md](BUILD_OPTIMIZATION.md) | 构建提速：mold 链接器 / ccache / colcon 并行度等配置与实测 |
| [CPP20_REFACTORING_ROADMAP.md](CPP20_REFACTORING_ROADMAP.md) | C++20 现代化重构路线（`<numbers>` / designated initializers / `std::format` / `span` / `ranges` 等） |
| [DEFECT_DIFFERENTIAL.md](DEFECT_DIFFERENTIAL.md) | 缺陷差分记录：ROS1 与 ROS2 行为差异导致的缺陷及修复证据 |
| [BENCH_CALIBRATION_TEMPLATE.md](BENCH_CALIBRATION_TEMPLATE.md) | benchmark 基线重校准模板（`benchmarks/baseline/` 口径与流程） |

## 子目录

- `agents/`：面向 AI agent 的协作辅助文档（issue 标签映射等）。

## 相关入口

- 决策台账：`.agent/DECISIONS.md`（长期决策 D-001 起，含 pending 项）。
- 项目总览与本地验证回路：仓库根 `README.md` 与 `CLAUDE.md`。
