// SteeringCalibration 编解码往返一致性测试（issue #2：统一 Pure Pursuit / MPC / 仿真器的转向编码与零位）
#include <gtest/gtest.h>

#include "steering_calibration.h"  // 仓库约定：common_msgs 手写头不带前缀（同 cone_types.h）

using common_msgs::vehicle::SteeringCalibration;

TEST(SteeringCalibrationTest, DefaultProtocolMatchesSimulatorConvention) {
    const SteeringCalibration calib;
    EXPECT_EQ(calib.neutralRaw(), 90);
    // 零转角：控制器输出与仿真/评测解码一致解释为零
    EXPECT_EQ(calib.encodeRad(0.0), 90);
    EXPECT_DOUBLE_EQ(calib.decodeRad(90), 0.0);
}

TEST(SteeringCalibrationTest, KnownAngleRoundTripWithinQuantization) {
    const SteeringCalibration calib;
    for (double deg : {-25.0, -10.0, -1.0, 1.0, 10.0, 25.0}) {
        const double rad = deg * (SteeringCalibration::kPi / 180.0);
        const int raw = calib.encodeRad(rad);
        const double decoded = calib.decodeRad(raw);
        // 1 raw/度 ⇒ 量化误差不超过 0.5°
        EXPECT_NEAR(decoded, rad, 0.5 * (SteeringCalibration::kPi / 180.0)) << "deg=" << deg;
        EXPECT_EQ(raw, 90 + static_cast<int>(deg));
    }
}

TEST(SteeringCalibrationTest, SignConventionIsPositiveToTheRightLikeSimulator) {
    const SteeringCalibration calib;
    // 正转角（raw > 零位）解码后仍为正
    EXPECT_GT(calib.decodeRad(calib.encodeRad(0.2)), 0.0);
    EXPECT_LT(calib.decodeRad(calib.encodeRad(-0.2)), 0.0);
}

TEST(SteeringCalibrationTest, EncodingClampsBeyondPhysicalRange) {
    const SteeringCalibration calib;
    EXPECT_EQ(calib.encodeRad(10.0), 115);
    EXPECT_EQ(calib.encodeRad(-10.0), 65);
    // clamp 后解码不超出 ±25°
    EXPECT_NEAR(calib.decodeRad(200), 25.0 * (SteeringCalibration::kPi / 180.0), 1e-9);
    EXPECT_NEAR(calib.decodeRad(-200), -25.0 * (SteeringCalibration::kPi / 180.0), 1e-9);
}

TEST(SteeringCalibrationTest, LegacyAlternativeProtocolRoundTrips) {
    // 真实底盘若为旧 ROS1 风格（零位 110、3.73 raw/度、0..220）：仅参数化，无需改代码
    const SteeringCalibration chassis{
        .neutral = 110.0,
        .units_per_degree = 3.73,
        .min_raw = 0.0,
        .max_raw = 220.0,
    };
    EXPECT_EQ(chassis.encodeRad(0.0), 110);
    EXPECT_DOUBLE_EQ(chassis.decodeRad(110), 0.0);
    const double rad = 10.0 * (SteeringCalibration::kPi / 180.0);
    const double decoded = chassis.decodeRad(chassis.encodeRad(rad));
    // 3.73 raw/度 ⇒ 量化误差不超过约 0.134°
    EXPECT_NEAR(decoded, rad, 0.5 / 3.73 * (SteeringCalibration::kPi / 180.0));
}
