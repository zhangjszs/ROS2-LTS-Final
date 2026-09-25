// issue #15：跨消息边界的指令往返一致性测试。
// 覆盖既有 test_vehicle_command_codec 未触及的一段链路：
//   物理量 (转角 rad / 加速度 m/s²)
//     → SteeringCalibration::encodeRad + ActuatorCalibration::encode（共用标定层）
//     → VehicleCommandEncoder 组装 common_msgs::msg::HuatVehicleCmd（含帧头/校验和）
//     → 仿真器侧解码路径（SteeringCalibration::decodeRad + ActuatorCalibration::decode）
//   断言：整链量化误差有界、非有限输入落到安全制动、帧头/校验和可被消费者验证。
// 这是"仿真替身在无车条件下验证接口"（#15 验收第 3 条）的机读门禁。
#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "pure_pursuit/vehicle_command_encoder.h"
#include "steering_calibration.h"
#include "vehicle_command_codec.h"

using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::SteeringCalibration;
using common_msgs::vehicle::VehicleCommandRaw;
using common_msgs::vehicle::verifyChecksum;
using common_msgs::vehicle::verifyFrame;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

// 与 PP / MPC / 仿真器三方参数默认值一致（sim 协议：零位 90，1 raw/度，±25°）。
SteeringCalibration simSteering() {
    return SteeringCalibration{.neutral = 90.0, .units_per_degree = 1.0, .min_raw = 65.0, .max_raw = 115.0};
}

// 把消息的指令区投影回共用层的纯数据视图（消费者侧解码前的实际形态）。
VehicleCommandRaw rawOf(const common_msgs::msg::HuatVehicleCmd& m) {
    VehicleCommandRaw r;
    r.steering = m.steering;
    r.brake_force = m.brake_force;
    r.pedal_ratio = m.pedal_ratio;
    r.gear_position = m.gear_position;
    r.working_mode = m.working_mode;
    r.racing_num = m.racing_num;
    r.racing_status = m.racing_status;
    return r;
}
}  // namespace

// 转角往返：rad → raw → rad，误差 < 一个 raw 单位对应的角度。
TEST(CommandRoundTrip, SteeringAngleSurvivesMessageBoundary) {
    const auto steer = simSteering();
    const auto act = ActuatorCalibration{};
    VehicleCommandEncoder enc(steer.neutralRaw());

    for (double deg : {-25.0, -10.0, -0.5, 0.0, 0.5, 10.0, 25.0}) {
        const double rad = deg * M_PI / 180.0;
        const auto cmd = enc.encodeDrive(steer.encodeRad(rad), act.encode(0.0).pedal, /*racing_num=*/1,
                                         /*racing_status=*/1);
        const double back = steer.decodeRad(cmd.steering);
        // 1 raw 单位 = 1/units_per_degree 度
        const double tol = (1.0 / steer.units_per_degree) * M_PI / 180.0;
        EXPECT_NEAR(back, rad, tol) << "deg=" << deg;
    }
}

// 加速度往返：m/s² → (油门|制动) raw → m/s²，误差有界（量化截断，非环绕）。
TEST(CommandRoundTrip, AccelerationSurvivesMessageBoundary) {
    const auto steer = simSteering();
    const auto act = ActuatorCalibration{};
    VehicleCommandEncoder enc(steer.neutralRaw());

    for (double a : {0.0, 1.0, 2.5, 5.0, -1.0, -4.0, -8.0}) {
        const auto tb = act.encode(a);
        const auto cmd = enc.encode(steer.neutralRaw(), tb.brake, tb.pedal, 1, 1, 1, 1);
        const double back = act.decode(cmd.pedal_ratio, cmd.brake_force);
        const double tol = std::max(act.max_accel, act.max_decel) / act.pedal_full_scale + 1e-9;
        EXPECT_NEAR(back, a, tol) << "a=" << a;
    }
}

// 非有限目标加速度：整链落到"无油门 + 安全制动"，出口字节合法且 checksum/frame 可验证。
TEST(CommandRoundTrip, NonFiniteAccelerationBecomesSafeBrakeOnTheWire) {
    const auto steer = simSteering();
    const auto act = ActuatorCalibration{};
    VehicleCommandEncoder enc(steer.neutralRaw());

    for (double bad : {kNaN, kInf, -kInf}) {
        const auto tb = act.encode(bad);
        ASSERT_TRUE(tb.safe_fallback) << "input=" << bad;
        const auto cmd = enc.encode(steer.neutralRaw(), tb.brake, tb.pedal, 1, 1, 1, 1);
        EXPECT_EQ(cmd.pedal_ratio, 0u) << "input=" << bad;
        EXPECT_EQ(cmd.brake_force, act.emergencyBrakeRaw()) << "input=" << bad;
        // 消费者（仿真器 / #16 仲裁）能验证这一帧，且解码得到有界负加速度而非乱值
        EXPECT_TRUE(verifyFrame(cmd.head1, cmd.head2, cmd.length)) << "input=" << bad;
        EXPECT_TRUE(verifyChecksum(rawOf(cmd), cmd.checksum)) << "input=" << bad;
        const double decoded = act.decode(cmd.pedal_ratio, cmd.brake_force);
        EXPECT_TRUE(std::isfinite(decoded)) << "input=" << bad;
        EXPECT_LE(decoded, 0.0) << "input=" << bad;
    }
}

// 帧被篡改（载荷位翻转）→ 消费者必须能检出（#16 仲裁信任门的解码侧对偶）。
TEST(CommandRoundTrip, TamperedFrameIsDetectedByConsumer) {
    const auto steer = simSteering();
    const auto act = ActuatorCalibration{};
    VehicleCommandEncoder enc(steer.neutralRaw());

    const auto tb = act.encode(2.5);
    auto cmd = enc.encode(steer.neutralRaw(), tb.brake, tb.pedal, 1, 1, 1, 1);
    ASSERT_TRUE(verifyChecksum(rawOf(cmd), cmd.checksum));
    cmd.pedal_ratio = static_cast<uint8_t>(cmd.pedal_ratio + 1);  // 线上被改写
    EXPECT_FALSE(verifyChecksum(rawOf(cmd), cmd.checksum));
}

// 0–255 满量程底盘替身：只改标定值即可切换，无需改代码；往返仍在满量程内。
TEST(CommandRoundTrip, TwoHundredFiftyFiveScaleDecoderAgreement) {
    SteeringCalibration wide{.neutral = 128.0, .units_per_degree = 2.0, .min_raw = 78.0, .max_raw = 178.0};
    ActuatorCalibration act;
    act.pedal_full_scale = 255.0;
    act.calibration_version = "vcu-255-probe";
    VehicleCommandEncoder enc(wide.neutralRaw());

    const auto tb = act.encode(act.max_accel);  // 满油门
    EXPECT_EQ(tb.pedal, 255);
    const auto cmd = enc.encode(wide.encodeRad(0.1), tb.brake, tb.pedal, 1, 1, 1, 1);
    EXPECT_EQ(cmd.pedal_ratio, 255u);
    EXPECT_TRUE(verifyChecksum(rawOf(cmd), cmd.checksum));
    EXPECT_NEAR(act.decode(cmd.pedal_ratio, cmd.brake_force), act.max_accel,
                act.max_accel / act.pedal_full_scale + 1e-9);
    // 转角：0.1 rad ≈ 5.73° → raw = 128 + 5.73*2 ≈ 139（截断）
    EXPECT_GE(cmd.steering, 138u);
    EXPECT_LE(cmd.steering, 140u);
    EXPECT_NEAR(wide.decodeRad(cmd.steering), 0.1, (1.0 / wide.units_per_degree) * M_PI / 180.0);
}

// checksumRaw 只覆盖指令区：帧头/length 不参与累加和，故两者必须一起验。
TEST(CommandRoundTrip, FrameHeadIsNotCoveredByChecksumSoMustBeCheckedSeparately) {
    const auto steer = simSteering();
    VehicleCommandEncoder enc(steer.neutralRaw());
    auto cmd = enc.encode(steer.neutralRaw(), 0, 50, 1, 1, 1, 1);
    const auto good = checksumRaw(rawOf(cmd));
    cmd.head1 = 0x00;  // 整帧错位：校验和依旧自洽
    EXPECT_TRUE(verifyChecksum(rawOf(cmd), good));
    EXPECT_FALSE(verifyFrame(cmd.head1, cmd.head2, cmd.length));
}
