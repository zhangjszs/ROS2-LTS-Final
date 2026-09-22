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
