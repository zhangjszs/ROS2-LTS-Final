#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "track_benchmark/kpi_evaluator.hpp"
#include "track_benchmark/track_generator.hpp"

using namespace benchmark;

TEST(TrackGeneratorTest, SkidpadTrackGeneration) {
    auto track = TrackGenerator::GenerateSkidpad();
    EXPECT_EQ(track.name, "Skidpad");
    EXPECT_GT(track.cones.size(), 80u);
    EXPECT_GT(track.centerline.size(), 50u);
    EXPECT_NEAR(track.track_width, 3.0, 1e-4);
}

TEST(TrackGeneratorTest, TrackdriveLoopGeneration) {
    auto track = TrackGenerator::GenerateTrackdriveLoop(35.0, 20.0, 3.0, 100);
    EXPECT_EQ(track.name, "Trackdrive_Loop");
    EXPECT_GT(track.cones.size(), 80u);
    EXPECT_GT(track.total_length, 100.0);
}

// —— #17 单一赛道来源：赛道身份与几何校验和不得悄悄漂移 ——

TEST(TrackRegistry, EachTrackHasStableIdVersionAndClosedFlag) {
    const auto accel = TrackGenerator::GenerateAcceleration();
    const auto skid = TrackGenerator::GenerateSkidpad();
    const auto loop = TrackGenerator::GenerateTrackdriveLoop();

    EXPECT_EQ(accel.id, "acceleration-75m");
    EXPECT_EQ(skid.id, "skidpad-figure8");
    EXPECT_EQ(loop.id, "trackdrive-loop");
    // 直线赛不得被当作闭合赛道（计圈/越界判据依赖它，消费者不得再靠名字猜）。
    EXPECT_FALSE(accel.closed_circuit);
    EXPECT_TRUE(skid.closed_circuit);
    EXPECT_TRUE(loop.closed_circuit);
    for (const auto& t : {accel, skid, loop}) {
        EXPECT_EQ(t.version, "v1");
        EXPECT_EQ(t.versionedId(), t.id + "/" + t.version);
        EXPECT_FALSE(t.name.empty());
    }
}

// 几何校验和锁定：改动生成器几何而不递增 version 会在这里失败（而不是产出一堆难以归因的基线飘移）。
TEST(TrackRegistry, GeometryChecksumIsFixedForCommittedVersion) {
    EXPECT_EQ(TrackGenerator::geometryChecksum(TrackGenerator::GenerateAcceleration().cones), 0x64291c72658ad63fULL);
    EXPECT_EQ(TrackGenerator::geometryChecksum(TrackGenerator::GenerateSkidpad().cones), 0x3b098a8899d93303ULL);
    EXPECT_EQ(TrackGenerator::geometryChecksum(TrackGenerator::GenerateTrackdriveLoop().cones), 0x5549f6279f3b456bULL);
}

// 同一几何两次求和一致，不同赛道则不一致：校验和可当“同一赛道”的等同判据。
TEST(TrackRegistry, ChecksumDistinguishesTracksAndIsDeterministic) {
    const auto a = TrackGenerator::GenerateAcceleration();
    const auto b = TrackGenerator::GenerateAcceleration();
    const auto c = TrackGenerator::GenerateSkidpad();
    EXPECT_EQ(TrackGenerator::geometryChecksum(a.cones), TrackGenerator::geometryChecksum(b.cones));
    EXPECT_NE(TrackGenerator::geometryChecksum(a.cones), TrackGenerator::geometryChecksum(c.cones));
}

TEST(KpiEvaluatorTest, StraightLineTrackingError) {
    std::vector<CenterlinePoint> centerline;
    for (double x = 0.0; x <= 50.0; x += 1.0) {
        centerline.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .s = x});
    }

    KpiEvaluator eval(centerline);

    // 赛车保持 y = 0.20m (右偏/左偏固定误差)
    for (int i = 0; i < 20; ++i) {
        eval.Update(static_cast<double>(i), 0.20, 0.0, 5.0, 0.0, static_cast<double>(i) * 0.1);
    }

    auto summary = eval.GetSummary();
    EXPECT_NEAR(summary.rmse_lateral_m, 0.20, 1e-3);
    EXPECT_NEAR(summary.max_lateral_error_m, 0.20, 1e-3);
    EXPECT_NEAR(summary.mean_lateral_error_m, 0.20, 1e-3);
}

TEST(KpiEvaluatorTest, PerfectTrackingZeroRmse) {
    std::vector<CenterlinePoint> centerline;
    for (double x = 0.0; x <= 20.0; x += 1.0) {
        centerline.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .s = x});
    }

    KpiEvaluator eval(centerline);

    // 赛车完全精确行走在中心线上
    for (int i = 0; i < 15; ++i) {
        eval.Update(static_cast<double>(i), 0.0, 0.0, 5.0, 0.0, static_cast<double>(i) * 0.1);
    }

    auto summary = eval.GetSummary();
    EXPECT_NEAR(summary.rmse_lateral_m, 0.0, 1e-4);
    EXPECT_NEAR(summary.max_lateral_error_m, 0.0, 1e-4);
}

TEST(KpiEvaluatorTest, MarkdownReportFormatting) {
    KpiSummary summary{.track_name = "Skidpad",
                       .controller_name = "PurePursuit",
                       .rmse_lateral_m = 0.125,
                       .max_lateral_error_m = 0.28,
                       .mean_lateral_error_m = 0.09,
                       .peak_lat_accel_g = 0.75,
                       .peak_lon_accel_g = 0.50,
                       .max_speed_mps = 8.5,
                       .avg_speed_mps = 6.0,
                       .current_lap_time_s = 14.2,
                       .best_lap_time_s = 13.8,
                       .completed_laps = 2,
                       .cone_collisions = 0,
                       .steering_jerk = 1.2,
                       .total_samples = 500};

    std::string md = KpiEvaluator::GenerateMarkdownReport(summary);
    EXPECT_NE(md.find("FSAC 控制算法基准性能评估报告"), std::string::npos);
    EXPECT_NE(md.find("0.125 m"), std::string::npos);
    EXPECT_NE(md.find("0.75 g"), std::string::npos);
}

// —— #17 C：判据正确性 —— //

namespace {
// 构造半径 R 的闭合圆周参考线（逆时针），s 为弧长
std::vector<CenterlinePoint> MakeCircle(double R, int n) {
    std::vector<CenterlinePoint> cl;
    for (int i = 0; i < n; ++i) {
        double a = 2.0 * std::numbers::pi_v<double> * static_cast<double>(i) / n;
        cl.push_back({.x = R * std::cos(a),
                      .y = R * std::sin(a),
                      .theta = a + std::numbers::pi_v<double> / 2.0,
                      .curvature = 1.0 / R,
                      .s = R * a});
    }
    return cl;
}
struct CircleEval {
    double R{10.0};
    double total{2.0 * std::numbers::pi_v<double> * 10.0};
    KpiEvaluator make(bool closed = true, double corridor = 1.5) {
        KpiEvaluator e(MakeCircle(R, 200));
        e.SetCircuitGeometry(total, corridor, closed);
        return e;
    }
};
}  // namespace

TEST(KpiEvaluatorTest, ClosedLoopForwardCountsOneValidLap) {
    CircleEval ce;
    auto eval = ce.make(true);
    int n = 200;
    for (int i = 0; i <= n; ++i) {
        double a = 2.0 * std::numbers::pi_v<double> * i / n;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 5.0, 0.0, i * 0.05);  // ~10s 一圈
    }
    auto s = eval.GetSummary();
    EXPECT_GE(s.valid_laps, 1);
    EXPECT_GT(s.best_valid_lap_time_s, 0.0);
    EXPECT_DOUBLE_EQ(s.best_lap_time_s, s.best_valid_lap_time_s);
    // 正向：净弧长进度为一个正的全圈长（#17：方向作为可正值识别的判据）
    EXPECT_GT(s.net_arc_progress_m, 0.5 * ce.total);
}

TEST(KpiEvaluatorTest, ReverseTraversalCountsNoLap) {
    CircleEval ce;
    auto eval = ce.make(true);
    int n = 200;
    for (int i = 0; i <= n; ++i) {  // 反向：角度递减
        double a = -2.0 * std::numbers::pi_v<double> * i / n;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 5.0, 0.0, i * 0.05);
    }
    auto s = eval.GetSummary();
    EXPECT_EQ(s.valid_laps, 0);
    EXPECT_DOUBLE_EQ(s.best_lap_time_s, 0.0);  // 未完赛不产出最佳圈速
    // 反向经过起点：不仅“不计圈”，净进度必须为负（评测能正面识别跑错方向）
    EXPECT_LT(s.net_arc_progress_m, -0.5 * ce.total);
}

TEST(KpiEvaluatorTest, UnfinishedNoBestLap) {
    CircleEval ce;
    auto eval = ce.make(true);
    int n = 200;
    for (int i = 0; i < n / 2; ++i) {  // 只跑半圈
        double a = 2.0 * std::numbers::pi_v<double> * i / n;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 5.0, 0.0, i * 0.05);
    }
    auto s = eval.GetSummary();
    EXPECT_EQ(s.valid_laps, 0);
    EXPECT_DOUBLE_EQ(s.best_lap_time_s, 0.0);
}

TEST(KpiEvaluatorTest, ConeCollisionIsEnvelopeAndDeduplicated) {
    CircleEval ce;
    auto eval = ce.make(false);        // 不关圈速，专注碰撞
    std::vector<BenchmarkCone> cones;  // 在参考线上放一个锥桶（角度 0）
    cones.push_back({.x = ce.R, .y = 0.0, .type = 1, .id = 0});
    eval.SetTrackCones(cones);
    // 从锥桶附近多帧掠过 -> 应去重为 1 次事件
    for (int i = -5; i <= 5; ++i) {
        double a = i * 0.01;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 3.0, 0.0, (i + 5) * 0.1);
    }
    auto s = eval.GetSummary();
    EXPECT_EQ(s.collision_events, 1);
    EXPECT_EQ(s.collision_cones, 1);
    EXPECT_EQ(s.cone_collisions, 1);  // 旧字段同步为去重后事件数
}

TEST(KpiEvaluatorTest, EnvelopeMissesConeJustOutsideWidth) {
    CircleEval ce;
    auto eval = ce.make(false);
    std::vector<BenchmarkCone> cones;  // 横向偏离 1.2m（默认车宽1.0/2+锥桶0.1=0.6 阈值外）
    cones.push_back({.x = ce.R, .y = 1.2, .type = 1, .id = 0});
    eval.SetTrackCones(cones);
    for (int i = -3; i <= 3; ++i) {
        double a = i * 0.01;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 3.0, 0.0, (i + 3) * 0.1);
    }
    EXPECT_EQ(eval.GetSummary().collision_events, 0);
}

TEST(KpiEvaluatorTest, OutOfBoundsCorridorEvents) {
    CircleEval ce;
    auto eval = ce.make(false, 1.5);  // 走廊半宽 1.5
    // 横向偏离 3m（>1.5）连续多帧 -> 越界事件为 1（边沿）且样本>0
    for (int i = 0; i < 10; ++i) {
        double a = i * 0.05;
        eval.Update((ce.R + 3.0) * std::cos(a), (ce.R + 3.0) * std::sin(a), a, 3.0, 0.0, i * 0.1);
    }
    auto s = eval.GetSummary();
    EXPECT_EQ(s.out_of_bounds_events, 1);
    EXPECT_GT(s.out_of_bounds_samples, 0u);
}

TEST(KpiEvaluatorTest, LateralAccelSourceLabelledAsReference) {
    CircleEval ce;
    auto eval = ce.make(false);
    double a = 0.0;
    eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 5.0, 0.0, 0.0);
    EXPECT_EQ(eval.GetSummary().lat_accel_source, "reference_curvature");
}

// —— #17 终态判定（闭环节点/离线 runner 共用）：把累计 KPI 落成 finished/timed_out/run_status ——

TEST(KpiTerminalStatusTest, TimeoutOverridesEverything) {
    KpiSummary s{};
    s.valid_laps = 3;  // 即便已有有效圈，超时也优先判 timeout（不得因有圈记成 finished）
    ApplyTerminalStatus(s, /*timed_out=*/true, /*require_laps=*/1);
    EXPECT_TRUE(s.timed_out);
    EXPECT_FALSE(s.finished);
    EXPECT_EQ(s.run_status, "timeout");
}

TEST(KpiTerminalStatusTest, ValidLapFinishesZeroLapIncomplete) {
    KpiSummary finished{};
    finished.valid_laps = 1;
    ApplyTerminalStatus(finished, false, 0);
    EXPECT_TRUE(finished.finished);
    EXPECT_FALSE(finished.timed_out);
    EXPECT_EQ(finished.run_status, "finished");

    KpiSummary dnf{};  // 未完赛：valid_laps=0 且不得产出最佳圈速
    ApplyTerminalStatus(dnf, false, 0);
    EXPECT_FALSE(dnf.finished);
    EXPECT_EQ(dnf.run_status, "incomplete");
    EXPECT_DOUBLE_EQ(dnf.best_valid_lap_time_s, 0.0);
}

TEST(KpiTerminalStatusTest, RequireLapsThresholdIsEnforced) {
    KpiSummary one{};
    one.valid_laps = 1;
    ApplyTerminalStatus(one, false, /*require_laps=*/2);  // 要求 2 圈但只跑 1 圈 -> 未完赛
    EXPECT_FALSE(one.finished);
    EXPECT_EQ(one.run_status, "incomplete");

    KpiSummary two{};
    two.valid_laps = 2;
    ApplyTerminalStatus(two, false, /*require_laps=*/2);
    EXPECT_TRUE(two.finished);
    EXPECT_EQ(two.run_status, "finished");
}

TEST(KpiEvaluatorTest, JsonReportMachineReadableFields) {
    CircleEval ce;
    auto eval = ce.make(true);
    eval.SetTrackVersion("trackdrive/v1");
    int n = 200;
    for (int i = 0; i <= n; ++i) {
        double a = 2.0 * std::numbers::pi_v<double> * i / n;
        eval.Update(ce.R * std::cos(a), ce.R * std::sin(a), a, 5.0, 0.0, i * 0.05);
    }
    std::string json = KpiEvaluator::GenerateJsonReport(eval.GetSummary());
    EXPECT_EQ(json.front(), '{');
    EXPECT_EQ(json[json.find_last_not_of(" \n\r\t")], '}');  // 去除尾部空白后以 } 结尾
    EXPECT_NE(json.find("\"schema\": \"fsac.benchmark.kpi/v1\""), std::string::npos);
    EXPECT_NE(json.find("\"track_version\": \"trackdrive/v1\""), std::string::npos);
    EXPECT_NE(json.find("\"valid_laps\":"), std::string::npos);
    EXPECT_NE(json.find("\"best_valid_lap_time_s\":"), std::string::npos);
    EXPECT_NE(json.find("\"lat_accel_source\": \"reference_curvature\""), std::string::npos);
    EXPECT_NE(json.find("\"net_arc_progress_m\":"), std::string::npos);  // #17 方向判据可机读
    // 括号平衡（基本结构校验）
    size_t open = 0, close = 0;
    for (char ch : json) {
        if (ch == '{')
            ++open;
        if (ch == '}')
            ++close;
    }
    EXPECT_EQ(open, close);
}
