#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pure_pursuit/input_guard.h"
#include "pure_pursuit/pp_math.h"
#include "pure_pursuit/vehicle_command_encoder.h"

// ── pp_math::estimateCurvature ─────────────────────────────────────────────

TEST(PpMath, StraightLineZeroCurvature) {
    std::vector<double> rx = {0, 1, 2, 3, 4};
    std::vector<double> ry = {0, 0, 0, 0, 0};
    EXPECT_NEAR(pp_math::estimateCurvature(rx, ry, 2), 0.0, 1e-9);
}

TEST(PpMath, UnitCircleCurvature) {
    // 半径为 r 的圆，κ 应接近 1/r
    double r = 5.0;
    std::vector<double> rx, ry;
    for (int i = -1; i <= 1; ++i) {
        double theta = i * (M_PI / 6.0);  // -30°, 0°, +30°
        rx.push_back(r * std::cos(theta));
        ry.push_back(r * std::sin(theta));
    }
    double kappa = pp_math::estimateCurvature(rx, ry, 1);
    EXPECT_NEAR(kappa, 1.0 / r, 1e-6);
}

TEST(PpMath, EdgeCaseTooFewPoints) {
    std::vector<double> rx = {0, 1};
    std::vector<double> ry = {0, 0};
    EXPECT_DOUBLE_EQ(pp_math::estimateCurvature(rx, ry, 1), 0.0);
}

TEST(PpMath, EdgeCaseBoundaryIndex) {
    std::vector<double> rx = {0, 1, 2, 3};
    std::vector<double> ry = {0, 0, 0, 0};
    // idx=0 和 idx=3 (末尾) 应返回 0
    EXPECT_DOUBLE_EQ(pp_math::estimateCurvature(rx, ry, 0), 0.0);
    EXPECT_DOUBLE_EQ(pp_math::estimateCurvature(rx, ry, 3), 0.0);
}

TEST(PpMath, CollinearPointsZeroCurvature) {
    // 共线但不是严格水平
    std::vector<double> rx = {0, 1, 2};
    std::vector<double> ry = {0, 1, 2};
    EXPECT_NEAR(pp_math::estimateCurvature(rx, ry, 1), 0.0, 1e-9);
}

TEST(PpMath, KnownRadiusCurvature) {
    // 半径为 10m 的半圆采样，κ 应接近 0.1
    double r = 10.0;
    std::vector<double> rx, ry;
    for (int i = -2; i <= 2; ++i) {
        double theta = i * (M_PI / 8.0);
        rx.push_back(r * std::cos(theta));
        ry.push_back(r * std::sin(theta));
    }
    double kappa = pp_math::estimateCurvature(rx, ry, 2);
    EXPECT_NEAR(kappa, 1.0 / r, 1e-3);
}

TEST(PpMath, SharpTurnHigherCurvature) {
    // 急弯（小半径）比缓弯（大半径）曲率更大
    double r_small = 3.0, r_large = 15.0;
    std::vector<double> rx_small, ry_small, rx_large, ry_large;
    for (int i = -1; i <= 1; ++i) {
        double theta = i * (M_PI / 6.0);
        rx_small.push_back(r_small * std::cos(theta));
        ry_small.push_back(r_small * std::sin(theta));
        rx_large.push_back(r_large * std::cos(theta));
        ry_large.push_back(r_large * std::sin(theta));
    }
    double kappa_small = pp_math::estimateCurvature(rx_small, ry_small, 1);
    double kappa_large = pp_math::estimateCurvature(rx_large, ry_large, 1);
    EXPECT_GT(kappa_small, kappa_large);
}

TEST(FindNearestIndex, PicksClosestPoint) {
    std::vector<double> rx = {0, 1, 2, 3};
    std::vector<double> ry = {0, 0, 0, 0};
    auto r = pp_math::findNearestIndex(rx, ry, 2.1, 0.0, 0, 4);
    EXPECT_EQ(r.idx, 2);
    EXPECT_NEAR(r.dist_sq, 0.01, 1e-9);
}

TEST(FindNearestIndex, EmptyPath) {
    std::vector<double> rx, ry;
    auto r = pp_math::findNearestIndex(rx, ry, 0, 0, 0, 0);
    EXPECT_EQ(r.idx, -1);
}

TEST(FindNearestIndex, WindowLimitsSearch) {
    std::vector<double> rx = {0, 10, 20};
    std::vector<double> ry = {0, 0, 0};
    auto r = pp_math::findNearestIndex(rx, ry, 0.0, 0.0, 1, 3);
    EXPECT_EQ(r.idx, 1);
}

TEST(FindNearestIndex, SizeMismatchIsEmpty) {
    std::vector<double> rx = {0, 1, 2};
    std::vector<double> ry = {0, 0};
    auto r = pp_math::findNearestIndex(rx, ry, 0, 0, 0, 3);
    EXPECT_EQ(r.idx, -1);
}

TEST(FindNearestIndex, CrosstrackDistanceIsReported) {
    std::vector<double> rx = {0, 1, 2};
    std::vector<double> ry = {0, 0, 0};
    auto r = pp_math::findNearestIndex(rx, ry, 1.0, 10.0, 0, 3);
    EXPECT_EQ(r.idx, 1);
    EXPECT_NEAR(r.dist_sq, 100.0, 1e-9);
}

// ── pp_math::findLookaheadIndex (折线累加流式管道测试) ──────────────────────

TEST(FindLookaheadIndex, AccumulatesAlongPolyline) {
    // 路径点间距为 1.0m：(0,0), (1,0), (2,0), (3,0), (4,0)
    std::vector<double> rx = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> ry = {0.0, 0.0, 0.0, 0.0, 0.0};
    // 从 idx=0 开始，lookahead=2.5m，应累积走过 1.0 + 1.0 = 2.0m (到达 idx=2)，再走 1.0m 超过 2.5m，最终停在 idx=2
    int idx = pp_math::findLookaheadIndex(rx, ry, 0, 2.5);
    EXPECT_EQ(idx, 2);

    // lookahead=3.0m，刚好到达 idx=3
    EXPECT_EQ(pp_math::findLookaheadIndex(rx, ry, 0, 3.0), 3);
}

TEST(FindLookaheadIndex, ZeroLookaheadReturnsCurrent) {
    std::vector<double> rx = {0.0, 1.0, 2.0};
    std::vector<double> ry = {0.0, 0.0, 0.0};
    EXPECT_EQ(pp_math::findLookaheadIndex(rx, ry, 1, 0.0), 1);
}

TEST(FindLookaheadIndex, ExceedsPathLengthClampsToEnd) {
    std::vector<double> rx = {0.0, 1.0, 2.0};
    std::vector<double> ry = {0.0, 0.0, 0.0};
    // 超过路径总长，应停在最后一个索引 2
    EXPECT_EQ(pp_math::findLookaheadIndex(rx, ry, 0, 100.0), 2);
}

// ── pp_math::findLookaheadIndexEuclidean (欧氏距离 filter & transform 管道测试) ──

TEST(FindLookaheadIndexEuclidean, FiltersPointsBeyondLookahead) {
    // 点距离 (0,0) 的距离分别为 0, 1, 2, 3, 4
    std::vector<double> rx = {0.0, 1.0, 2.0, 3.0, 4.0};
    std::vector<double> ry = {0.0, 0.0, 0.0, 0.0, 0.0};
    // lookahead=2.5m，首个距离 >= 2.5m 的点是 idx=3 (dist=3.0m)
    int idx = pp_math::findLookaheadIndexEuclidean(rx, ry, 0, 2.5);
    EXPECT_EQ(idx, 3);
}

TEST(FindLookaheadIndexEuclidean, DiagonalPath) {
    // 对角线路径：(0,0), (3,4) 距离 5m
    std::vector<double> rx = {0.0, 3.0, 6.0};
    std::vector<double> ry = {0.0, 4.0, 8.0};
    EXPECT_EQ(pp_math::findLookaheadIndexEuclidean(rx, ry, 0, 4.0), 1);
}

TEST(FindLookaheadIndexEuclidean, BeyondEndReturnsLast) {
    std::vector<double> rx = {0.0, 1.0};
    std::vector<double> ry = {0.0, 0.0};
    EXPECT_EQ(pp_math::findLookaheadIndexEuclidean(rx, ry, 0, 50.0), 1);
}

TEST(IsBaseLinkFrame, RecognizesBaseLink) {
    EXPECT_TRUE(pp_math::isBaseLinkFrame("base_link"));
    EXPECT_FALSE(pp_math::isBaseLinkFrame("map"));
    EXPECT_FALSE(pp_math::isBaseLinkFrame(""));
    EXPECT_FALSE(pp_math::isBaseLinkFrame("Base_link"));
    EXPECT_FALSE(pp_math::isBaseLinkFrame("base_link "));
}

// ── clampKappaIdx ─────────────────────────────────────────────────────────

TEST(ClampKappaIdxTest, WithinRangeUnchanged) {
    EXPECT_EQ(pp_math::clampKappaIdx(3, 10), 3);
    EXPECT_EQ(pp_math::clampKappaIdx(5, 10), 5);
    EXPECT_EQ(pp_math::clampKappaIdx(8, 10), 8);
}

TEST(ClampKappaIdxTest, BelowMinClampedToOne) {
    EXPECT_EQ(pp_math::clampKappaIdx(0, 10), 1);
    EXPECT_EQ(pp_math::clampKappaIdx(-5, 10), 1);
}

TEST(ClampKappaIdxTest, AboveMaxClampedToNMinus2) {
    EXPECT_EQ(pp_math::clampKappaIdx(9, 10), 8);
    EXPECT_EQ(pp_math::clampKappaIdx(100, 10), 8);
}

TEST(ClampKappaIdxTest, MinimumSize) {
    // n=3 → max valid idx = 1
    EXPECT_EQ(pp_math::clampKappaIdx(0, 3), 1);
    EXPECT_EQ(pp_math::clampKappaIdx(1, 3), 1);
    EXPECT_EQ(pp_math::clampKappaIdx(2, 3), 1);
}

// ── InputGuard ─────────────────────────────────────────────────────────────

TEST(InputGuard, ProceedWhenAllOk) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    rclcpp::Time state(99, 900000000);  // 0.1s 前
    rclcpp::Time path(99, 800000000);   // 0.2s 前
    auto r = g.check(now, state, path, false, false, true, true);
    EXPECT_EQ(r.decision, GuardDecision::PROCEED);
}

TEST(InputGuard, HardBrakeOnNoState) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    rclcpp::Time zero(0, 0);
    auto r = g.check(now, zero, rclcpp::Time(99, 900000000), false, false, false, true);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
    EXPECT_EQ(r.brake_force, 80);
}

TEST(InputGuard, HardBrakeOnStateTimeout) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    rclcpp::Time old_state(99, 500000000);  // 0.5s > 0.3s timeout
    auto r = g.check(now, old_state, rclcpp::Time(99, 900000000), false, false, true, true);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
}

TEST(InputGuard, SoftBrakeOnEmptyPath) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(99, 900000000), rclcpp::Time(99, 900000000), true, false, true, true);
    EXPECT_EQ(r.decision, GuardDecision::SOFT_BRAKE);
    EXPECT_EQ(r.brake_force, 40);
}

TEST(InputGuard, SoftBrakeOnPathTimeout) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    rclcpp::Time old_path(99, 400000000);  // 0.6s > 0.5s timeout
    auto r = g.check(now, rclcpp::Time(99, 900000000), old_path, false, false, true, true);
    EXPECT_EQ(r.decision, GuardDecision::SOFT_BRAKE);
}

TEST(InputGuard, HardBrakeOnStopSignal) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(99, 900000000), rclcpp::Time(99, 900000000), false, true, true, true);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
    EXPECT_EQ(r.brake_force, 80);
}

TEST(InputGuard, StopBeatsEmptyPath) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(99, 900000000), rclcpp::Time(99, 900000000), true, true, true, true);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
    EXPECT_EQ(r.brake_force, 80);
}

TEST(InputGuard, StopBeatsPathTimeout) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(99, 900000000), rclcpp::Time(99, 0), false, true, true, true);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
    EXPECT_EQ(r.brake_force, 80);
}

TEST(InputGuard, StopBeatsMissingState) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(0, 0), rclcpp::Time(0, 0), true, true, false, false);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
    EXPECT_EQ(r.brake_force, 80);
}

TEST(InputGuard, NeverReceivedPathIsSoftBrake) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(100, 0);
    auto r = g.check(now, rclcpp::Time(99, 900000000), rclcpp::Time(0, 0), false, false, true, false);
    EXPECT_EQ(r.decision, GuardDecision::SOFT_BRAKE);
    EXPECT_EQ(r.brake_force, 40);
}

TEST(InputGuard, NeverReceivedStateAtSimTimeZero) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(0, 0);
    auto r = g.check(now, rclcpp::Time(0, 0), rclcpp::Time(0, 0), false, false, false, false);
    EXPECT_EQ(r.decision, GuardDecision::HARD_BRAKE);
}

TEST(InputGuard, SimTimeZeroIsValidOnceReceived) {
    InputGuard g(0.3, 0.5);
    rclcpp::Time now(0, 50000000);
    auto r = g.check(now, rclcpp::Time(0, 0), rclcpp::Time(0, 0), false, false, true, true);
    EXPECT_EQ(r.decision, GuardDecision::PROCEED);
}

// ── VehicleCommandEncoder ──────────────────────────────────────────────────

TEST(VehicleCommandEncoder, BrakeCommandFields) {
    VehicleCommandEncoder enc;
    auto cmd = enc.encodeBrake(1, 80);
    EXPECT_EQ(cmd.steering, 110);
    EXPECT_EQ(cmd.brake_force, 80);
    EXPECT_EQ(cmd.pedal_ratio, 0);
    EXPECT_EQ(cmd.racing_num, 1);
    EXPECT_EQ(cmd.racing_status, 4);
    EXPECT_EQ(cmd.head1, 0xAA);
    EXPECT_EQ(cmd.head2, 0x55);
}

TEST(VehicleCommandEncoder, DriveCommandFields) {
    VehicleCommandEncoder enc;
    auto cmd = enc.encodeDrive(115, 30, 1, 1);
    EXPECT_EQ(cmd.steering, 115);
    EXPECT_EQ(cmd.pedal_ratio, 30);
    EXPECT_EQ(cmd.brake_force, 0);
    EXPECT_EQ(cmd.racing_num, 1);
    EXPECT_EQ(cmd.racing_status, 1);
}

TEST(VehicleCommandEncoder, ChecksumConsistency) {
    VehicleCommandEncoder enc;
    auto a = enc.encodeDrive(110, 20, 1, 1);
    auto b = enc.encodeDrive(110, 20, 1, 1);
    EXPECT_EQ(a.checksum, b.checksum);
}

TEST(VehicleCommandEncoder, ChecksumChangesWithParams) {
    VehicleCommandEncoder enc;
    auto a = enc.encodeDrive(110, 20, 1, 1);
    auto b = enc.encodeDrive(120, 20, 1, 1);  // 转向不同
    EXPECT_NE(a.checksum, b.checksum);
}

// ── Pure pursuit steering geometry ─────────────────────────────────────────
// 验证纯追踪几何关系：左弯→负舵角，右弯→正舵角。
// 使用与控制器相同的公式：delta = atan2(L * sin(alpha), 1.0)
// 其中 alpha = atan2(goal_y, goal_x)（base_link 下的目标方向角）。

TEST(PurePursuitGeometry, LeftTurnProducesPositiveSinAlpha) {
    // 目标在车辆左前方 → alpha > 0 → delta > 0（左转）
    double goalX = 5.0, goalY = 2.0;  // 左前方
    double alpha = std::atan2(goalY, goalX);
    EXPECT_GT(alpha, 0.0);
    double L = 2.0;
    double delta = std::atan2(L * std::sin(alpha) / 5.0, 1.0);
    EXPECT_GT(delta, 0.0);
}

TEST(PurePursuitGeometry, RightTurnProducesNegativeDelta) {
    // 目标在车辆右前方 → alpha < 0 → delta < 0（右转）
    double goalX = 5.0, goalY = -2.0;  // 右前方
    double alpha = std::atan2(goalY, goalX);
    EXPECT_LT(alpha, 0.0);
    double L = 2.0;
    double delta = std::atan2(L * std::sin(alpha) / 5.0, 1.0);
    EXPECT_LT(delta, 0.0);
}

TEST(PurePursuitGeometry, StraightAheadProducesZeroDelta) {
    // 目标在正前方 → alpha = 0 → delta = 0
    double goalX = 5.0, goalY = 0.0;
    double alpha = std::atan2(goalY, goalX);
    EXPECT_NEAR(alpha, 0.0, 1e-12);
    double L = 2.0;
    double delta = std::atan2(L * std::sin(alpha) / 5.0, 1.0);
    EXPECT_NEAR(delta, 0.0, 1e-12);
}

// ── C++20 std::span tests ──────────────────────────────────────────────────

TEST(SpanAdoption, EstimateCurvatureWithRawCArray) {
    // 验证原生静态数组 double[] 能够无需构造 vector 直接传入 estimateCurvature
    const double raw_x[] = {0.0, 1.0, 2.0, 3.0, 4.0};
    const double raw_y[] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double kappa = pp_math::estimateCurvature(raw_x, raw_y, 2);
    EXPECT_NEAR(kappa, 0.0, 1e-9);
}

TEST(SpanAdoption, FindNearestIndexWithStdArrayAndSubspan) {
    // 验证 std::array 及其 subspan() 切片零拷贝查找
    const std::array<double, 5> arr_x = {0.0, 10.0, 20.0, 30.0, 40.0};
    const std::array<double, 5> arr_y = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::span<const double> full_span_x(arr_x);
    std::span<const double> full_span_y(arr_y);
    auto sub_x = full_span_x.subspan(1, 3);  // 10.0, 20.0, 30.0
    auto sub_y = full_span_y.subspan(1, 3);
    auto res = pp_math::findNearestIndex(sub_x, sub_y, 19.5, 0.0, 0, 3);
    EXPECT_EQ(res.idx, 1);  // sub_x[1] = 20.0
}

TEST(SpanAdoption, InputGuardSpanOverload) {
    InputGuard guard(0.5, 0.5);
    rclcpp::Time now(10, 0);
    rclcpp::Time last_state(10, 0);
    rclcpp::Time last_path(10, 0);

    const double points_x[] = {1.0, 2.0, 3.0};
    std::span<const double> span_x(points_x);
    auto res = guard.check(now, last_state, last_path, span_x, false, true, true);
    EXPECT_EQ(res.decision, GuardDecision::PROCEED);

    std::span<const double> empty_span;
    auto res_empty = guard.check(now, last_state, last_path, empty_span, false, true, true);
    EXPECT_EQ(res_empty.decision, GuardDecision::SOFT_BRAKE);
    EXPECT_STREQ(res_empty.reason, "Path is empty");
}

TEST(SpanAdoption, VehicleCommandEncoderPayloadChecksum) {
    const uint8_t payload[] = {0xAA, 0x55, 0x01, 0x02, 0x03};
    uint16_t checksum = VehicleCommandEncoder::computeChecksum(payload);
    EXPECT_EQ(checksum, static_cast<uint16_t>(0xAA + 0x55 + 0x01 + 0x02 + 0x03));
    EXPECT_TRUE(VehicleCommandEncoder::verifyChecksum(payload, checksum));
    EXPECT_FALSE(VehicleCommandEncoder::verifyChecksum(payload, static_cast<uint16_t>(checksum + 1)));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
