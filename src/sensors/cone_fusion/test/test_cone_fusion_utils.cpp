#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "cone_types.h"
#include "cone_fusion_utils.h"

using cone_fusion_utils::ConfidenceToPercent;
using cone_fusion_utils::IsVehicleStateJumpAbnormal;
using cone_fusion_utils::NormalizeAngle;
using cone_fusion_utils::VehicleState;

constexpr double kPi = std::numbers::pi_v<double>;

// ── NormalizeAngle ─────────────────────────────────────────────────────────

TEST(NormalizeAngleTest, AlreadyInRange) {
    EXPECT_NEAR(NormalizeAngle(0.0), 0.0, 1e-12);
    EXPECT_NEAR(NormalizeAngle(kPi / 2), kPi / 2, 1e-12);
    EXPECT_NEAR(NormalizeAngle(-kPi / 2), -kPi / 2, 1e-12);
}

TEST(NormalizeAngleTest, WrapsPositive) {
    EXPECT_NEAR(NormalizeAngle(3 * kPi), kPi, 1e-12);
    EXPECT_NEAR(NormalizeAngle(2 * kPi + 0.1), 0.1, 1e-12);
    EXPECT_NEAR(NormalizeAngle(5 * kPi / 2), kPi / 2, 1e-12);
}

TEST(NormalizeAngleTest, WrapsNegative) {
    EXPECT_NEAR(NormalizeAngle(-3 * kPi), -kPi, 1e-12);
    EXPECT_NEAR(NormalizeAngle(-2 * kPi - 0.1), -0.1, 1e-12);
    EXPECT_NEAR(NormalizeAngle(-5 * kPi / 2), -kPi / 2, 1e-12);
}

TEST(NormalizeAngleTest, BoundaryValues) {
    EXPECT_NEAR(NormalizeAngle(kPi), kPi, 1e-12);
    EXPECT_NEAR(NormalizeAngle(-kPi), -kPi, 1e-12);
}

// ── ConfidenceToPercent ────────────────────────────────────────────────────

TEST(ConfidenceToPercentTest, NormalizesFractional) {
    float data[] = {0.0f, 0.5f, 1.0f};
    EXPECT_EQ(ConfidenceToPercent(data, 3, 0), 0u);
    EXPECT_EQ(ConfidenceToPercent(data, 3, 1), 50u);
    EXPECT_EQ(ConfidenceToPercent(data, 3, 2), 100u);
}

TEST(ConfidenceToPercentTest, AlreadyInPercent) {
    float data[] = {0.0f, 50.0f, 100.0f};
    EXPECT_EQ(ConfidenceToPercent(data, 3, 0), 0u);
    EXPECT_EQ(ConfidenceToPercent(data, 3, 1), 50u);
    EXPECT_EQ(ConfidenceToPercent(data, 3, 2), 100u);
}

TEST(ConfidenceToPercentTest, OutOfRangeIndex) {
    float data[] = {0.5f};
    EXPECT_EQ(ConfidenceToPercent(data, 1, 5), 0u);
}

TEST(ConfidenceToPercentTest, ClampsOverflow) {
    float data[] = {1.5f, -0.5f, 150.0f};
    // 1.5 is > 1.0 so not scaled; passes through as lround(1.5) = 2
    EXPECT_EQ(ConfidenceToPercent(data, 3, 0), 2u);
    // negative clamps to 0
    EXPECT_EQ(ConfidenceToPercent(data, 3, 1), 0u);
    // > 100 clamps to 100
    EXPECT_EQ(ConfidenceToPercent(data, 3, 2), 100u);
}

TEST(ConfidenceToPercentTest, HandlesNaN) {
    float data[] = {std::nanf("")};
    EXPECT_EQ(ConfidenceToPercent(data, 1, 0), 0u);
}

TEST(ConfidenceToPercentTest, EmptyArray) {
    EXPECT_EQ(ConfidenceToPercent(nullptr, 0, 0), 0u);
}

// ── IsVehicleStateJumpAbnormal ─────────────────────────────────────────────

// Default test parameters
static constexpr double kBaseJumpThreshold = 2.0;
static constexpr double kSpeedMargin = 2.0;
static constexpr double kMinDt = 0.02;
static constexpr double kMaxDt = 1.0;
static constexpr double kHeadingThreshold = 0.5;

TEST(IsVehicleStateJumpTest, NormalSmallMovementIsNotAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 0.5, .y = 0.0, .theta = 0.01, .v = 5.0};
    std::string reason;
    // dt=0.1s, distance=0.5m, allowed = 2.0 + 5*0.1*2 = 3.0m
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(IsVehicleStateJumpTest, LargeDistanceJumpIsAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 10.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    std::string reason;
    // dt=0.1s, distance=10.0m, allowed = 2.0 + 5*0.1*2 = 3.0m
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_TRUE(result);
    EXPECT_FALSE(reason.empty());
}

TEST(IsVehicleStateJumpTest, LargeHeadingJumpIsAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 0.0, .y = 0.0, .theta = 1.0, .v = 5.0};
    std::string reason;
    // heading_jump=1.0 > threshold=0.5
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_TRUE(result);
}

TEST(IsVehicleStateJumpTest, ZeroDtIsNotAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 100.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    std::string reason;
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.0, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(IsVehicleStateJumpTest, NegativeDtIsNotAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 100.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    std::string reason;
    bool result = IsVehicleStateJumpAbnormal(last, curr, -0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(IsVehicleStateJumpTest, ExceedsMaxDtIsNotAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 100.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    std::string reason;
    bool result = IsVehicleStateJumpAbnormal(last, curr, 2.0, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(IsVehicleStateJumpTest, HighSpeedAllowsLargerJump) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 30.0};
    VehicleState curr{.x = 5.0, .y = 0.0, .theta = 0.0, .v = 30.0};
    std::string reason;
    // dt=0.1s, distance=5.0m, allowed = 2.0 + 30*0.1*2 = 8.0m → not abnormal
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(IsVehicleStateJumpTest, HighSpeedExcessiveJumpIsAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 30.0};
    VehicleState curr{.x = 20.0, .y = 0.0, .theta = 0.0, .v = 30.0};
    std::string reason;
    // dt=0.1s, distance=20.0m, allowed = 2.0 + 30*0.1*2 = 8.0m → abnormal
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_TRUE(result);
}

TEST(IsVehicleStateJumpTest, ReasonStringPopulatedOnAbnormal) {
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    VehicleState curr{.x = 10.0, .y = 0.0, .theta = 0.0, .v = 5.0};
    std::string reason;
    IsVehicleStateJumpAbnormal(last, curr, 0.1, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt, kHeadingThreshold,
                               &reason);
    EXPECT_NE(reason.find("distance_jump="), std::string::npos);
    EXPECT_NE(reason.find("heading_jump="), std::string::npos);
}

TEST(IsVehicleStateJumpTest, MinDtClamping) {
    // When dt < min_dt, min_dt is used for the speed-dependent term
    VehicleState last{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 30.0};
    VehicleState curr{.x = 2.5, .y = 0.0, .theta = 0.0, .v = 30.0};
    std::string reason;
    // dt=0.001 < min_dt=0.02, bounded_dt=0.02
    // allowed = 2.0 + 30*0.02*2 = 3.2m, distance=2.5m → not abnormal
    bool result = IsVehicleStateJumpAbnormal(last, curr, 0.001, kBaseJumpThreshold, kSpeedMargin, kMinDt, kMaxDt,
                                             kHeadingThreshold, &reason);
    EXPECT_FALSE(result);
}

TEST(ConeTypeEnum, ColorDoesNotCollideWithLidarSizeMeaning) {
    EXPECT_EQ(huat_cone::BLUE, 0u);
    EXPECT_EQ(huat_cone::YELLOW_SMALL, 1u);
    EXPECT_EQ(huat_cone::YELLOW_BIG, 2u);
    EXPECT_EQ(huat_cone::RED, 3u);
    EXPECT_EQ(huat_cone::NONE, 4u);
    EXPECT_EQ(huat_cone::SIZE_UNKNOWN, 0);
    EXPECT_EQ(huat_cone::SIZE_LARGE, 1);
    EXPECT_EQ(huat_cone::SIZE_SMALL, 2);
}

TEST(MergeVisionColorWithLidarSize, LeavesBlueAndRedAlone) {
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::BLUE, huat_cone::SIZE_LARGE), huat_cone::BLUE);
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::RED, huat_cone::SIZE_SMALL), huat_cone::RED);
}

TEST(MergeVisionColorWithLidarSize, SplitsYellowByLidarSize) {
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::YELLOW, huat_cone::SIZE_LARGE),
              huat_cone::YELLOW_BIG);
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::YELLOW, huat_cone::SIZE_SMALL),
              huat_cone::YELLOW_SMALL);
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::YELLOW, huat_cone::SIZE_UNKNOWN), huat_cone::YELLOW);
}

TEST(MergeVisionColorWithLidarSize, UnknownVisionIsNone) {
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(huat_cone::NONE, huat_cone::SIZE_LARGE), huat_cone::NONE);
    EXPECT_EQ(huat_cone::MergeVisionColorWithLidarSize(99, huat_cone::SIZE_SMALL), huat_cone::NONE);
}

TEST(MergeVisionColorWithLidarSize, LidarSizeIsNotAColor) {
    // Copying SIZE_LARGE (1) into HuatCone.type would look like YELLOW, not BLUE.
    EXPECT_NE(static_cast<uint32_t>(huat_cone::SIZE_LARGE), huat_cone::BLUE);
    EXPECT_EQ(static_cast<uint32_t>(huat_cone::SIZE_LARGE), huat_cone::YELLOW_SMALL);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
