// #80：直线赛完赛判据单测（终点线判定 / 停车区判定 / 到达速度上限反推）。
#include <gtest/gtest.h>

#include <cmath>

#include "track_benchmark/finish_judge.hpp"
#include "track_benchmark/track_generator.hpp"

using namespace benchmark;

namespace {

// 直线赛默认判据参数：终点线 75 m、缓冲区末端 100 m、制动 9 m/s²。
FinishRunoutSpec AccelSpec() {
    FinishRunoutSpec s;
    s.enabled = true;
    s.finish_x = 75.0;
    s.runout_end_m = 100.0;
    s.brake_decel = 9.0;
    s.stop_speed_eps = 0.05;
    s.safety = 0.75;
    return s;
}

}  // namespace

// —— 到达速度上限（由“越线后必须在缓冲区内停稳”反推）——

TEST(FinishJudgeTest, SpeedCapIsZeroWhenTrackHasNoFinishSemantics) {
    FinishRunoutSpec s;  // enabled=false（闭合赛道）
    EXPECT_DOUBLE_EQ(approachSpeedCap(s), 0.0);
}

TEST(FinishJudgeTest, SpeedCapIsZeroWhenRunoutDistanceIsNotPositive) {
    FinishRunoutSpec s = AccelSpec();
    s.runout_end_m = s.finish_x;  // 无缓冲距离
    EXPECT_DOUBLE_EQ(approachSpeedCap(s), 0.0);
    s.runout_end_m = s.finish_x - 1.0;
    EXPECT_DOUBLE_EQ(approachSpeedCap(s), 0.0);
}

TEST(FinishJudgeTest, SpeedCapFollowsRunoutDistanceAndBrakeCapability) {
    const FinishRunoutSpec s = AccelSpec();
    // v² = 2·a·d·safety → sqrt(2·9·25·0.75) = 18.3716
    const double expected = std::sqrt(2.0 * 9.0 * 25.0 * 0.75);
    EXPECT_NEAR(approachSpeedCap(s), expected, 1e-9);

    // 缓冲距离翻倍 → 速度上限按 sqrt 关系上升；制动能力减半 → 速度上限按 sqrt 关系下降
    FinishRunoutSpec longer = s;
    longer.runout_end_m = s.finish_x + 50.0;
    EXPECT_NEAR(approachSpeedCap(longer), expected * std::sqrt(2.0), 1e-9);

    FinishRunoutSpec weaker = s;
    weaker.brake_decel = s.brake_decel / 2.0;
    EXPECT_NEAR(approachSpeedCap(weaker), expected / std::sqrt(2.0), 1e-9);
}

TEST(FinishJudgeTest, SpeedCapIsEnoughToStopInsideRunoutAndSafeWhenNot) {
    const FinishRunoutSpec s = AccelSpec();
    const double cap = approachSpeedCap(s);
    ASSERT_GT(cap, 0.0);
    // 上限速度下的理论制动距离必须落在缓冲区内（含安全系数余量）
    EXPECT_LT(cap * cap / (2.0 * s.brake_decel), s.runout_end_m - s.finish_x);
    // 不设安全系数时制动距离恰好用尽缓冲距离（零余量）——这正是必须留余量的原因
    const double unsafe_cap = std::sqrt(2.0 * s.brake_decel * (s.runout_end_m - s.finish_x));
    EXPECT_NEAR(unsafe_cap * unsafe_cap / (2.0 * s.brake_decel), s.runout_end_m - s.finish_x, 1e-9);
}

// —— 终点线判定 ——

TEST(FinishJudgeTest, CrossingIsDetectedExactlyAtFinishLine) {
    FinishJudge judge(AccelSpec());
    EXPECT_FALSE(judge.Update(74.99, 20.0, 1.0));
    EXPECT_FALSE(judge.crossed());
    EXPECT_DOUBLE_EQ(judge.cross_time_s(), 0.0);

    EXPECT_TRUE(judge.Update(75.0, 20.0, 1.5));  // 恰好在 x=75 越线
    EXPECT_TRUE(judge.crossed());
    EXPECT_DOUBLE_EQ(judge.cross_time_s(), 1.5);

    // 越线只触发一次
    EXPECT_FALSE(judge.Update(76.0, 19.0, 1.6));
    EXPECT_DOUBLE_EQ(judge.cross_time_s(), 1.5);
}

TEST(FinishJudgeTest, DisabledJudgeNeverCrosses) {
    FinishRunoutSpec s;  // enabled=false
    FinishJudge judge(s);
    EXPECT_FALSE(judge.Update(100.0, 10.0, 1.0));
    EXPECT_FALSE(judge.crossed());
    EXPECT_FALSE(judge.IsStopped(100.0, 0.0));
    EXPECT_FALSE(judge.ExceededRunout(200.0));
}

// —— 停车区判定 ——

TEST(FinishJudgeTest, StopRequiresCrossingFirst) {
    FinishJudge judge(AccelSpec());
    judge.Update(70.0, 5.0, 1.0);
    // 还没越线：即便速度已归零也不算停稳（不得把“起步失败”误判成完赛）
    EXPECT_FALSE(judge.IsStopped(70.0, 0.0));
}

TEST(FinishJudgeTest, StopRequiresSpeedBelowEpsilon) {
    FinishJudge judge(AccelSpec());
    judge.Update(80.0, 15.0, 2.0);
    ASSERT_TRUE(judge.crossed());
    EXPECT_FALSE(judge.IsStopped(90.0, 0.5));   // 速度仍高于 ε
    EXPECT_FALSE(judge.IsStopped(90.0, 0.05));  // 恰好等于 ε 不算停稳（严格小于）
    EXPECT_TRUE(judge.IsStopped(90.0, 0.049));
}

TEST(FinishJudgeTest, StopIsAllowedExactlyAtRunoutEnd) {
    FinishJudge judge(AccelSpec());
    judge.Update(80.0, 15.0, 2.0);
    ASSERT_TRUE(judge.crossed());
    EXPECT_TRUE(judge.IsStopped(100.0, 0.01));     // 恰好停在缓冲区末端
    EXPECT_FALSE(judge.IsStopped(100.001, 0.01));  // 越出缓冲区 → 不算停稳
}

TEST(FinishJudgeTest, ExceedingRunoutIsDetected) {
    FinishJudge judge(AccelSpec());
    judge.Update(80.0, 15.0, 2.0);
    ASSERT_TRUE(judge.crossed());
    EXPECT_FALSE(judge.ExceededRunout(100.0));
    EXPECT_TRUE(judge.ExceededRunout(100.5));
}

// —— 与赛道元数据的对接（终点位置不得硬编码）——

TEST(FinishJudgeTest, SpecIsBuiltFromTrackMetadataNotHardcoded) {
    const auto track = TrackGenerator::GenerateAcceleration();
    ASSERT_TRUE(track.hasFinish());
    ASSERT_TRUE(track.finish_x.has_value());
    ASSERT_TRUE(track.runout_end_m.has_value());

    FinishRunoutSpec s;
    s.enabled = track.hasFinish();
    s.finish_x = *track.finish_x;
    s.runout_end_m = *track.runout_end_m;
    s.brake_decel = 9.0;

    EXPECT_DOUBLE_EQ(s.finish_x, 75.0);
    EXPECT_DOUBLE_EQ(s.runout_end_m, 100.0);
    // 闭合赛道没有终点语义
    EXPECT_FALSE(TrackGenerator::GenerateSkidpad().hasFinish());
    EXPECT_FALSE(TrackGenerator::GenerateTrackdriveLoop().hasFinish());
}
