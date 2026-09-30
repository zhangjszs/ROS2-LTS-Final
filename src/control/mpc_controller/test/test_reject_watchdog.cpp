// #50：RejectWatchdog 纯逻辑单测（连续拒解计数 / 阈值锁入 / 恢复解除 / 禁用）。
// 该文件同时注册进 colcon（ament_add_gtest）与 tests/core_standalone（#40 覆盖审计）。
#include <gtest/gtest.h>

#include "mpc_controller/controller_health.hpp"

namespace {

using mpc::RejectWatchdog;

TEST(RejectWatchdog, StaysDarkBelowThreshold) {
    RejectWatchdog w(5);
    for (int i = 0; i < 4; ++i) {
        w.Record(false);
    }
    EXPECT_FALSE(w.rejecting());
    EXPECT_EQ(w.consecutive_failures(), 4u);
    EXPECT_EQ(w.total_failures(), 4u);
}

TEST(RejectWatchdog, LatchesAtThresholdAndCountsTotals) {
    RejectWatchdog w(3);
    w.Record(false);
    w.Record(false);
    w.Record(false);
    EXPECT_TRUE(w.rejecting());
    EXPECT_EQ(w.consecutive_failures(), 3u);
    // 继续失败：计数只增不减，rejecting 保持
    w.Record(false);
    EXPECT_TRUE(w.rejecting());
    EXPECT_EQ(w.consecutive_failures(), 4u);
    EXPECT_EQ(w.total_failures(), 4u);
}

TEST(RejectWatchdog, SingleSuccessClearsStreakAndReleases) {
    RejectWatchdog w(2);
    w.Record(false);
    w.Record(false);
    ASSERT_TRUE(w.rejecting());
    w.Record(true);
    EXPECT_FALSE(w.rejecting());
    EXPECT_EQ(w.consecutive_failures(), 0u);
    EXPECT_EQ(w.total_failures(), 2u);  // total 是累计量，不随恢复回退
    // 解除后重新积累：还差一拍（未到阈值不点亮）
    w.Record(false);
    EXPECT_FALSE(w.rejecting());
    w.Record(false);
    EXPECT_TRUE(w.rejecting());
}

TEST(RejectWatchdog, ThresholdZeroDisablesLatching) {
    RejectWatchdog w(0);
    for (int i = 0; i < 100; ++i) {
        w.Record(false);
    }
    EXPECT_FALSE(w.rejecting());
    EXPECT_EQ(w.consecutive_failures(), 100u);
    EXPECT_EQ(w.total_failures(), 100u);
}

TEST(RejectWatchdog, RaiseThresholdReleasesUntilReachedAgain) {
    RejectWatchdog w(2);
    w.Record(false);
    w.Record(false);
    ASSERT_TRUE(w.rejecting());
    w.SetThreshold(10);
    EXPECT_FALSE(w.rejecting());  // 当前连续数不满足新阈值 ⇒ 锁解除
    for (int i = 0; i < 7; ++i) {
        w.Record(false);  // 从 2 续到 9：仍不足
    }
    EXPECT_FALSE(w.rejecting());
    w.Record(false);  // 第 10 拍
    EXPECT_TRUE(w.rejecting());
}

TEST(RejectWatchdog, DefaultThresholdIsFiftyBeats) {
    const RejectWatchdog w;  // #50 实现建议：默认连续 50 拍（=1s @50Hz）
    EXPECT_EQ(w.threshold(), mpc::RejectWatchdog::kDefaultThreshold);
    EXPECT_EQ(w.threshold(), 50u);
    EXPECT_FALSE(w.rejecting());
}

}  // namespace
