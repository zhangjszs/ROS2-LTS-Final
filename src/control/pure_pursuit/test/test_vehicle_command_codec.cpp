// issue #15：底盘执行器编解码 + 纵向标定单元测试（纯 std，无 ROS context）。
#include <gtest/gtest.h>

#include <algorithm>
#include <limits>

#include "vehicle_command_codec.h"  // 仓库约定：common_msgs 手写头不带前缀

using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::VehicleCommandRaw;
using common_msgs::vehicle::verifyChecksum;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
}  // namespace

TEST(VehicleCommandCodec, FrameConstantsMatchSimProtocol) {
    EXPECT_EQ(common_msgs::vehicle::kCmdHead1, 0xAA);
    EXPECT_EQ(common_msgs::vehicle::kCmdHead2, 0x55);
    EXPECT_EQ(common_msgs::vehicle::kCmdLength, 10);
}

TEST(VehicleCommandCodec, ChecksumSumsCommandRegion) {
    VehicleCommandRaw r;
    r.steering = 90;
    r.brake_force = 0;
    r.pedal_ratio = 30;
    r.gear_position = 1;
    r.working_mode = 1;
    r.racing_num = 7;
    r.racing_status = 1;
    EXPECT_EQ(checksumRaw(r), static_cast<uint16_t>(90 + 0 + 30 + 1 + 1 + 7 + 1));
    EXPECT_TRUE(verifyChecksum(r, checksumRaw(r)));
    EXPECT_FALSE(verifyChecksum(r, static_cast<uint16_t>(checksumRaw(r) + 1)));
}

TEST(VehicleCommandCodec, AccelerationMapsMutuallyExclusiveThrottleOrBrake) {
    ActuatorCalibration cal;  // max_accel=5, max_decel=8, full=100
    const auto drive = cal.encode(2.5);
    EXPECT_EQ(drive.pedal, 50);  // 2.5/5*100
    EXPECT_EQ(drive.brake, 0);
    EXPECT_FALSE(drive.safe_fallback);

    const auto brake = cal.encode(-4.0);
    EXPECT_EQ(brake.pedal, 0);
    EXPECT_EQ(brake.brake, 50);  // 4/8*100

    const auto coast = cal.encode(0.0);
    EXPECT_EQ(coast.pedal, 0);
    EXPECT_EQ(coast.brake, 0);
}

// ROS1 #6 缺陷类回归：非有限输入必须降级为「无油门 + 安全制动」，绝不因负值/溢出得到 255。
TEST(VehicleCommandCodec, NonFiniteInputDegradesToSafeBrake) {
    ActuatorCalibration cal;
    for (double bad : {kNaN, kInf, -kInf}) {
        const auto out = cal.encode(bad);
        EXPECT_EQ(out.pedal, 0) << "input=" << bad;
        EXPECT_EQ(out.brake, cal.emergencyBrakeRaw()) << "input=" << bad;
        EXPECT_TRUE(out.safe_fallback) << "input=" << bad;
    }
}

// clamp-before-narrow：超大正/负加速度只饱和到满量程，不环绕成任意字节（尤其是 255）。
TEST(VehicleCommandCodec, OversizedInputSaturatesNotWrapsTo255) {
    ActuatorCalibration cal;  // pedal_full_scale=100
    EXPECT_EQ(cal.encode(1e6).pedal, 100);
    EXPECT_EQ(cal.encode(1e6).brake, 0);
    EXPECT_EQ(cal.encode(-1e6).brake, 100);
    EXPECT_EQ(cal.encode(-1e6).pedal, 0);

    // 0-255 满量程底盘：负加速度也绝不产生 255 之外的环绕
    ActuatorCalibration byte255;
    byte255.pedal_full_scale = 255.0;
    const auto v = byte255.encode(-1e9);
    EXPECT_EQ(v.brake, 255);
    EXPECT_EQ(v.pedal, 0);
}

TEST(VehicleCommandCodec, RoundTripEncodeDecode) {
    ActuatorCalibration cal;
    for (double a : {0.0, 1.0, 2.5, 5.0, -1.0, -4.0, -8.0}) {
        const auto e = cal.encode(a);
        const double back = cal.decode(e.pedal, e.brake);
        // 截断窄化 => 往返误差 < 一个字节对应的加速度增量
        const double tol = cal.max_accel / cal.pedal_full_scale + 1e-9;
        EXPECT_NEAR(back, a, std::max(tol, cal.max_decel / cal.pedal_full_scale));
    }
}

TEST(VehicleCommandCodec, CalibrationVersionDefaultsAndOverridable) {
    ActuatorCalibration cal;
    EXPECT_EQ(cal.calibration_version, "sim-default-0");
    cal.calibration_version = "vcu-2026-09-24";
    EXPECT_EQ(cal.calibration_version, "vcu-2026-09-24");
}
