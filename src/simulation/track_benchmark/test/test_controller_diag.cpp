// #44（#19 B2）：控制器诊断采集层的数值契约测试。
// 本文件同时注册在 colcon 侧与 tests/core_standalone（ASan+UBSan）—— #40 的覆盖审计要求二选一。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "track_benchmark/controller_diag.hpp"

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

[[nodiscard]] std::vector<double> MakeSorted1To100() {
    std::vector<double> v;
    v.reserve(100);
    for (int i = 1; i <= 100; ++i) {
        v.push_back(static_cast<double>(i));
    }
    return v;
}

}  // namespace

TEST(ControllerDiagTest, NearestRankPercentileMatchesHandComputation) {
    const auto v = MakeSorted1To100();
    // 最近秩 idx = ceil(q/100 · n)，1-based：n=100 时 p50=50、p95=95、p99=99
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(v, 50.0), 50.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(v, 95.0), 95.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(v, 99.0), 99.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(v, 100.0), 100.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(v, 0.0), 1.0);  // ceil(0)=0→夹到 1

    // n=10 时高分位向上跳到一个真实样本（不做插值）：p95=ceil(9.5)=10、p50=5
    std::vector<double> ten;
    for (int i = 1; i <= 10; ++i) {
        ten.push_back(static_cast<double>(i));
    }
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(ten, 50.0), 5.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(ten, 95.0), 10.0);
    EXPECT_DOUBLE_EQ(benchmark::ControllerDiagnostics::PercentileOfSorted(ten, 99.0), 10.0);
}

// 空样本不得伪造数字：一切统计为 0，由 sample_count 表达"无数据"。
TEST(ControllerDiagTest, EmptySamplesDoNotFabricateNumbers) {
    benchmark::ControllerDiagnostics diag;
    const auto s = diag.GetSnapshot();
    EXPECT_EQ(s.sample_count, 0u);
    EXPECT_EQ(s.solves, 0u);
    EXPECT_DOUBLE_EQ(s.failure_rate, 0.0);
    EXPECT_DOUBLE_EQ(s.p50_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.p95_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.p99_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.mean_ms, 0.0);
    EXPECT_DOUBLE_EQ(s.max_ms, 0.0);
}

// 未收敛率 + 每臂不变式 command_updates + holds == control_ticks。
TEST(ControllerDiagTest, FailureRateAndTickAccountingInvariant) {
    benchmark::ControllerDiagnostics diag;
    // 10 拍：7 次成功、3 次求解失败并因此保持上一条指令
    for (int i = 0; i < 10; ++i) {
        const bool ok = (i < 7);
        diag.RecordControlTick();
        diag.RecordSolve(static_cast<double>(i), ok);
        diag.RecordCommandUpdate();
        if (!ok) {
            diag.RecordHold();
            diag.RecordCommandUpdate();  // 上一次成功解仍在被采用，只为构造不变式反例场景
        }
    }
    auto s = diag.GetSnapshot();
    EXPECT_EQ(s.solves, 10u);
    EXPECT_EQ(s.failures, 3u);
    EXPECT_DOUBLE_EQ(s.failure_rate, 0.3);
    // 上面故意多记了一次 update（每次失败额外记），因此这里断言的是"计数彼此独立、
    // 不被内部偷偷耦合"：ticks=10, holds=3, updates=13。
    EXPECT_EQ(s.control_ticks, 10u);
    EXPECT_EQ(s.holds, 3u);
    EXPECT_EQ(s.command_updates, 13u);
    EXPECT_DOUBLE_EQ(s.p99_ms, 9.0);  // 样本 0..9 升序，ceil(0.99×10)=10 → 第 10 个 = 9.0
}

// 正确使用姿势（runner 的做法）：每 tick 要么 update 要么 hold，二者互斥。
TEST(ControllerDiagTest, RunnerStyleAccountingIsExclusiveAndPercentileIsRealSample) {
    benchmark::ControllerDiagnostics diag;
    for (int i = 0; i < 10; ++i) {
        const bool ok = (i < 7);
        diag.RecordControlTick();
        diag.RecordSolve(static_cast<double>(i), ok);
        if (ok) {
            diag.RecordCommandUpdate();
        } else {
            diag.RecordHold();
        }
    }
    const auto s = diag.GetSnapshot();
    EXPECT_EQ(s.command_updates + s.holds, s.control_ticks);
    EXPECT_EQ(s.holds, 3u);
    EXPECT_DOUBLE_EQ(s.failure_rate, 0.3);
    EXPECT_DOUBLE_EQ(s.mean_ms, 4.5);
    EXPECT_DOUBLE_EQ(s.max_ms, 9.0);
    // 百分位必须等于某个真实样本值（不插值造数）
    EXPECT_DOUBLE_EQ(s.p99_ms, 9.0);
    EXPECT_DOUBLE_EQ(s.p95_ms, 9.0);
    EXPECT_DOUBLE_EQ(s.p50_ms, 4.0);  // 样本是 0..9（不是 1..10）：ceil(0.5×10)=5 → 第 5 个 = 4.0
}

// 耗时非有限/为负 → 不进分布样本，但 solves/failures 仍如实计数。
TEST(ControllerDiagTest, NonFiniteElapsedExcludedFromSamplesButStillCounted) {
    benchmark::ControllerDiagnostics diag;
    diag.RecordSolve(kNaN, true);
    diag.RecordSolve(kInf, false);
    diag.RecordSolve(-1.0, true);
    diag.RecordSolve(2.0, true);
    const auto s = diag.GetSnapshot();
    EXPECT_EQ(s.solves, 4u);
    EXPECT_EQ(s.failures, 1u);
    EXPECT_EQ(s.sample_count, 1u);
    EXPECT_DOUBLE_EQ(s.p50_ms, 2.0);
    EXPECT_TRUE(std::isfinite(s.mean_ms));
}

// 产物的 schema、字段顺序与确定性；且不复用 KPI 字段（两份 schema 互不污染）。
TEST(ControllerDiagTest, JsonIsStableSeparateSchemaAndDeterministic) {
    benchmark::ControllerDiagnostics diag;
    diag.RecordControlTick();
    diag.RecordSolve(1.5, false);
    diag.RecordHold();
    const std::string a = diag.GenerateJson("trackdrive-loop/v1", "MPC", "curvature");
    const std::string b = diag.GenerateJson("trackdrive-loop/v1", "MPC", "curvature");
    EXPECT_EQ(a, b);
    EXPECT_NE(a.find("\"schema\": \"fsac.benchmark.controller_diag/v1\""), std::string::npos);
    EXPECT_NE(a.find("\"percentile_method\": \"nearest_rank_ceil\""), std::string::npos);
    EXPECT_NE(a.find("\"track_version\": \"trackdrive-loop/v1\""), std::string::npos);
    EXPECT_NE(a.find("\"failure_rate\""), std::string::npos);
    // 独立 schema：不得携带 KPI 字段（否则基线口径会被悄悄扩大）
    EXPECT_EQ(a.find("rmse_lateral_m"), std::string::npos);
    EXPECT_EQ(a.find("valid_laps"), std::string::npos);
    // 括号平衡（基本结构校验，与 KPI 报告同姿势）
    size_t open = 0, close = 0;
    for (const char c : a) {
        if (c == '{')
            ++open;
        if (c == '}')
            ++close;
    }
    EXPECT_EQ(open, close);
}

// #47：诊断必须能区分"真收敛"与"兜底带内接受的欠收敛解"。缺这两个计数时，
// "未收敛率 19%"会被读成"其余 81% 都收敛了"——实测默认参数下真收敛只约 16%。
TEST(ControllerDiagTest, AcceptanceBreakdownIsCountedAndSerialized) {
    benchmark::ControllerDiagnostics diag;
    using mpc::QpAcceptance;
    diag.RecordSolve(1.0, true, QpAcceptance::kConverged);
    diag.RecordSolve(2.0, true, QpAcceptance::kConverged);
    diag.RecordSolve(3.0, true, QpAcceptance::kAcceptedApproximation);
    diag.RecordSolve(4.0, false, QpAcceptance::kRejected);

    const auto s = diag.GetSnapshot();
    EXPECT_EQ(s.solves, 4u);
    EXPECT_EQ(s.failures, 1u);
    EXPECT_EQ(s.converged, 2u);
    EXPECT_EQ(s.accepted_approx, 1u);
    // 不变式：分解不重不漏
    EXPECT_EQ(s.failures + s.converged + s.accepted_approx, s.solves);

    const std::string json = diag.GenerateJson("trackdrive-loop/v1", "MPC", "curvature");
    EXPECT_NE(json.find("\"converged\": 2"), std::string::npos);
    EXPECT_NE(json.find("\"accepted_approx\": 1"), std::string::npos);
}

// 旧两参调用点（PP 臂/历史用例）行为不变：只计 failures，分解计数保持 0。
TEST(ControllerDiagTest, LegacyTwoArgCallsLeaveBreakdownZero) {
    benchmark::ControllerDiagnostics diag;
    diag.RecordSolve(1.5, true);
    diag.RecordSolve(2.5, false);

    const auto s = diag.GetSnapshot();
    EXPECT_EQ(s.solves, 2u);
    EXPECT_EQ(s.failures, 1u);
    EXPECT_EQ(s.converged, 0u);
    EXPECT_EQ(s.accepted_approx, 0u);
}
