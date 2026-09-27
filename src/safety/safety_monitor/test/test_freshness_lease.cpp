// issue #30：新鲜度租约语义单测（纯 std，可进 core_standalone）。
// 钉住四条：首帧 arming（Absent≠Stale）、超时边界、未来戳容限、<0 禁用语义。
#include <gtest/gtest.h>

#include <limits>

#include "freshness_lease.h"

using common_msgs::vehicle::Freshness;
using common_msgs::vehicle::FreshnessLease;
using common_msgs::vehicle::LeaseConfig;

TEST(FreshnessLeaseTest, AbsentBeforeFirstSample) {
    FreshnessLease lease(LeaseConfig{0.5, 0.0});
    EXPECT_FALSE(lease.hasSample());
    EXPECT_EQ(lease.check(100.0), Freshness::kAbsent);
    EXPECT_EQ(lease.age(100.0), std::numeric_limits<double>::infinity());
}

TEST(FreshnessLeaseTest, FreshWithinTimeout) {
    FreshnessLease lease(LeaseConfig{0.5, 0.0});
    lease.observe(100.0);
    EXPECT_TRUE(lease.hasSample());
    EXPECT_EQ(lease.check(100.4), Freshness::kFresh);
    EXPECT_NEAR(lease.age(100.4), 0.4, 1e-9);
}

TEST(FreshnessLeaseTest, BoundaryIsFresh) {
    FreshnessLease lease(LeaseConfig{0.5, 0.0});
    lease.observe(100.0);
    // 判定是 age > timeout：恰等于门限不算超时（与原 now-last > timeout 同语义）。
    EXPECT_EQ(lease.check(100.5), Freshness::kFresh);
    EXPECT_EQ(lease.check(100.500001), Freshness::kStale);
}

TEST(FreshnessLeaseTest, ReobserveRefreshes) {
    FreshnessLease lease(LeaseConfig{0.5, 0.0});
    lease.observe(100.0);
    lease.observe(100.6);
    EXPECT_EQ(lease.check(100.9), Freshness::kFresh);
}

TEST(FreshnessLeaseTest, FutureBeyondTolerance) {
    FreshnessLease lease(LeaseConfig{0.5, 0.1});
    lease.observe(100.2);
    EXPECT_EQ(lease.check(100.0), Freshness::kFromFuture);
}

TEST(FreshnessLeaseTest, FutureWithinToleranceIsFresh) {
    FreshnessLease lease(LeaseConfig{0.5, 0.5});
    lease.observe(100.2);
    EXPECT_EQ(lease.check(100.0), Freshness::kFresh);
}

TEST(FreshnessLeaseTest, NegativeTimeoutDisablesAgeCheck) {
    // 与 trustedAge <0 禁用同语义（#12）：只剩 Absent 判定。
    FreshnessLease lease(LeaseConfig{-1.0, 0.0});
    EXPECT_EQ(lease.check(100.0), Freshness::kAbsent);
    lease.observe(0.0);
    EXPECT_EQ(lease.check(1e9), Freshness::kFresh);
}

TEST(FreshnessLeaseTest, NegativeFutureToleranceTreatedAsZero) {
    FreshnessLease lease(LeaseConfig{0.5, -1.0});
    lease.observe(100.0);
    EXPECT_EQ(lease.check(99.999), Freshness::kFromFuture);
    EXPECT_EQ(lease.check(100.0), Freshness::kFresh);
}
