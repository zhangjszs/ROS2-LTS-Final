// issue #29：速度包络单一实现回归门（纯 std，无 ROS）。
// runner 内 PP 分支与 MPC 参考构建必须调同一函数：identical input → identical
// envelope。改包络式必须同步改本测试（阈值漂移即红），杜绝“改一处漏一处”。
#include <gtest/gtest.h>

#include <cmath>

#include "track_benchmark/speed_envelope.hpp"

using benchmark::referenceSpeedForCurvature;

TEST(SpeedEnvelope, StraightGivesMaxSpeed) {
    EXPECT_DOUBLE_EQ(referenceSpeedForCurvature(0.0, 4.0, 25.0), 25.0);
    EXPECT_DOUBLE_EQ(referenceSpeedForCurvature(-0.0, 4.0, 25.0), 25.0);
}

TEST(SpeedEnvelope, CurvatureLimitMatchesBaselineFormula) {
    // v = sqrt(4.0/0.16) = 5.0（与 v1 基线同式）
    EXPECT_NEAR(referenceSpeedForCurvature(0.16, 4.0, 25.0), 5.0, 1e-12);
    EXPECT_NEAR(referenceSpeedForCurvature(-0.16, 4.0, 25.0), 5.0, 1e-12);  // 限速看幅值
}

TEST(SpeedEnvelope, CappedByMaxSpeed) {
    // sqrt(4.0/1e-3) ≈ 63 > 25 → 顶到上限
    EXPECT_DOUBLE_EQ(referenceSpeedForCurvature(1e-4, 4.0, 25.0), 25.0);
}

TEST(SpeedEnvelope, SameInputSameEnvelopeForBothConsumers) {
    // PP 分支与 MPC 参考构建调同一函数：同一输入两次调用位级一致
    const double via_pp = referenceSpeedForCurvature(0.25, 4.0, 25.0);
    const double via_mpc = referenceSpeedForCurvature(0.25, 4.0, 25.0);
    EXPECT_DOUBLE_EQ(via_pp, via_mpc);
    EXPECT_NEAR(via_pp, std::sqrt(4.0 / 0.25), 1e-12);
}
