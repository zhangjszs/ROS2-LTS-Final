// SteeringCalibration 编解码往返一致性测试（issue #2：统一 Pure Pursuit / MPC / 仿真器的转向编码与零位）
// #38 起同时承担“非有限输入/误配标定”守卫回归，并注册进 tests/core_standalone（ASan+UBSan）。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

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

// ==================== #38：非有限输入与误配标定的安全守卫 ====================
// 本组用例把“转角指令要么合法、要么显式降级”固化为回归门；
// 它们同时被 tests/core_standalone（ASan+UBSan）跑一道，旧实现在那里会直接报 UB。

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

// 回归本 issue 的直接成因：非有限转角经 std::clamp(NaN) → lround(NaN) → raw 0，
// 再被解码端夹取成 -25° 满舵（而不是零位）。
TEST(SteeringCalibrationGuardTest, NonFiniteInputFallsBackToNeutralNotFullLock) {
    const SteeringCalibration calib;
    for (const double bad : {kNaN, kInf, -kInf}) {
        EXPECT_EQ(calib.encodeRad(bad), calib.neutralRaw()) << "input=" << bad;
        EXPECT_DOUBLE_EQ(calib.decodeRad(calib.encodeRad(bad)), 0.0) << "input=" << bad;
    }
}

// 降级必须可观测：合法输入不报错，非有限输入标 safe_fallback。
TEST(SteeringCalibrationGuardTest, EncodeRadCheckedReportsDegradation) {
    const SteeringCalibration calib;
    const auto legal = calib.encodeRadChecked(0.2);
    EXPECT_FALSE(legal.safe_fallback);
    EXPECT_EQ(legal.raw, calib.encodeRad(0.2));

    EXPECT_TRUE(calib.encodeRadChecked(kNaN).safe_fallback);
    EXPECT_TRUE(calib.encodeRadChecked(kInf).safe_fallback);
}

// 上下限写反（参数误配）不得再弄崩进程：旧实现在此触发 std::clamp 前置条件违反。
TEST(SteeringCalibrationGuardTest, InvertedRawRangeIsRejectedAndNeverCrashes) {
    SteeringCalibration swapped;
    swapped.min_raw = 115.0;
    swapped.max_raw = 65.0;
    EXPECT_FALSE(swapped.isConfigValid());
    // 不崩且仍产出字节内合法值；整对退回默认协议 ⇒ 与默认标定同值
    EXPECT_EQ(swapped.encodeRad(0.1), 96);
    EXPECT_EQ(swapped.encodeRad(10.0), 115);
    EXPECT_EQ(swapped.encodeRad(-10.0), 65);
    EXPECT_TRUE(std::isfinite(swapped.decodeRad(100)));
}

// 比例为 0/负/非有限 = 误配：不得让解码产出 inf。
TEST(SteeringCalibrationGuardTest, NonPositiveUnitsPerDegreeIsRejected) {
    for (const double bad : {0.0, -1.0, kNaN, kInf}) {
        SteeringCalibration calib;
        calib.units_per_degree = bad;
        EXPECT_FALSE(calib.isConfigValid()) << "units=" << bad;
        const double decoded = calib.decodeRad(100);
        EXPECT_TRUE(std::isfinite(decoded)) << "units=" << bad;
        EXPECT_NEAR(decoded, 10.0 * (SteeringCalibration::kPi / 180.0), 1e-9) << "units=" << bad;
    }
}

// 量程超出字节宽度（如 0–1000 的 VCU）：判定误配，且 clamp-before-narrow 保证不窄化回绕。
TEST(SteeringCalibrationGuardTest, ByteWidthOverflowSaturatesAndNeverWraps) {
    SteeringCalibration wide;
    wide.min_raw = -200.0;
    wide.max_raw = 300.0;
    EXPECT_FALSE(wide.isConfigValid());
    for (const double rad : {-10.0, -1.0, 0.0, 1.0, 10.0}) {
        const int raw = wide.encodeRad(rad);
        EXPECT_GE(raw, 0) << "rad=" << rad;
        EXPECT_LE(raw, 255) << "rad=" << rad;  // 旧行为：此处可得到 300 → static_cast<uint8_t> 静默回绕
    }
    EXPECT_EQ(wide.encodeRad(10.0), 255);
}

// 零位本身落在量程外 / 为非有限：不得让 neutralRaw() 跑出字节宽度。
TEST(SteeringCalibrationGuardTest, InvalidNeutralIsSanitizedIntoRange) {
    SteeringCalibration off;
    off.neutral = 200.0;  // 默认量程 [65,115] 之外
    EXPECT_FALSE(off.isConfigValid());
    EXPECT_EQ(off.neutralRaw(), 115);
    EXPECT_EQ(off.encodeRad(0.0), 115);

    SteeringCalibration nan_neutral;
    nan_neutral.neutral = kNaN;
    EXPECT_FALSE(nan_neutral.isConfigValid());
    EXPECT_EQ(nan_neutral.neutralRaw(), 90);  // 量程中点 = 默认协议零位
    EXPECT_EQ(nan_neutral.encodeRad(0.0), 90);
}

// 守卫不得改变合法标定的任何输出：对默认协议与旧 ROS1 风格协议逐值钉住。
TEST(SteeringCalibrationGuardTest, LegalCalibrationOutputsAreUnchanged) {
    const SteeringCalibration sim;
    // 旧公式：round(clamp(90 + deg, 65, 115))
    for (double deg = -25.0; deg <= 25.0; deg += 1.0) {
        const double rad = deg * (SteeringCalibration::kPi / 180.0);
        const int legacy = std::clamp(static_cast<int>(std::lround(90.0 + deg)), 65, 115);
        EXPECT_EQ(sim.encodeRad(rad), legacy) << "deg=" << deg;
    }

    const SteeringCalibration chassis{.neutral = 110.0, .units_per_degree = 3.73, .min_raw = 0.0, .max_raw = 220.0};
    EXPECT_TRUE(chassis.isConfigValid());
    for (double deg = -25.0; deg <= 25.0; deg += 1.0) {
        const double rad = deg * (SteeringCalibration::kPi / 180.0);
        const int legacy = std::clamp(static_cast<int>(std::lround(110.0 + deg * 3.73)), 0, 220);
        EXPECT_EQ(chassis.encodeRad(rad), legacy) << "deg=" << deg;
    }
}
