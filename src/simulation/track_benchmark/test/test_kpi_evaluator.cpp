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
