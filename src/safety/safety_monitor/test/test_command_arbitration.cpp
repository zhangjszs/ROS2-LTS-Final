// issue #16：最终指令仲裁器 —— 确定性故障注入矩阵（纯 std，无 ROS context，CI 门禁）。
// 覆盖：源超时/陈旧、数据无效、checksum 失败、stop 锁存、指令源冲突、无任一可信源降级。
#include <gtest/gtest.h>

#include <span>

#include "command_arbitration.h"  // 仓库约定：common_msgs 手写头不带前缀

using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::ArbitrationConfig;
using common_msgs::vehicle::ArbitrationReason;
using common_msgs::vehicle::ArbitrationResult;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::CommandArbitrator;
using common_msgs::vehicle::ControlSource;
using common_msgs::vehicle::SourceCandidate;
using common_msgs::vehicle::VehicleCommandRaw;

namespace {

constexpr std::uint8_t kNeutral = 90;

// 构造一个带正确 checksum 的可信候选指令。
SourceCandidate makeCandidate(ControlSource src, std::uint8_t steering, std::uint8_t pedal, std::uint8_t brake,
                              double age_sec) {
    SourceCandidate c;
    c.source = src;
    c.cmd.steering = steering;
    c.cmd.pedal_ratio = pedal;
    c.cmd.brake_force = brake;
    c.cmd.gear_position = 1;
    c.cmd.working_mode = 1;
    c.cmd.racing_num = 7;
    c.cmd.racing_status = 1;
    c.checksum = checksumRaw(c.cmd);
    c.age_sec = age_sec;
    c.present = true;
    return c;
}

CommandArbitrator makeArb() {
    return CommandArbitrator(ActuatorCalibration{}, kNeutral);
}

}  // namespace

TEST(CommandArbitration, SelectsPreferredControlSource) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::PURE_PURSUIT;
    auto pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 40, 0, 0.05);
    auto mpc = makeCandidate(ControlSource::MPC, 88, 30, 0, 0.05);
    const SourceCandidate cands[] = {pp, mpc};
    auto r = arb.decide(/*stop_active=*/false, /*can_drive=*/true, std::span{cands}, cfg);
    EXPECT_EQ(r.winner, ControlSource::PURE_PURSUIT);
    EXPECT_FALSE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::CONTROL);
    EXPECT_EQ(r.cmd.pedal_ratio, 40);
    EXPECT_EQ(r.checksum, checksumRaw(r.cmd));
}

// 故障注入：期望源 MPC 优先，但 PP 冲突时仍按 preferred 选 MPC。
TEST(CommandArbitration, ConflictResolvedByPreferred) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::MPC;
    auto pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 40, 0, 0.05);
    auto mpc = makeCandidate(ControlSource::MPC, 88, 30, 0, 0.05);
    const SourceCandidate cands[] = {pp, mpc};
    auto r = arb.decide(false, true, std::span{cands}, cfg);
    EXPECT_EQ(r.winner, ControlSource::MPC);
    EXPECT_EQ(r.cmd.steering, 88);
}

// 故障注入：源超时（age 超容差）→ 忽略该源，落到另一可信源；若全超时则安全降级。
TEST(CommandArbitration, StaleSourceIgnoredThenSafeFallback) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    cfg.source_timeout_sec = 0.5;
    cfg.preferred = ControlSource::PURE_PURSUIT;
    auto stale_pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 40, 0, 1.0);  // 1.0s > 0.5s → 陈旧
    auto fresh_mpc = makeCandidate(ControlSource::MPC, 88, 20, 0, 0.1);          // 新鲜
    const SourceCandidate c1[] = {stale_pp, fresh_mpc};
    auto r1 = arb.decide(false, true, std::span{c1}, cfg);
    EXPECT_EQ(r1.winner, ControlSource::MPC);  // preferred 陈旧 → 回退到另一可信源
    EXPECT_FALSE(r1.safe_fallback);

    const SourceCandidate c2[] = {stale_pp};
    auto r2 = arb.decide(false, true, std::span{c2}, cfg);
    EXPECT_TRUE(r2.safe_fallback);  // 仅剩陈旧源 → 无任一可信源，安全降级
    EXPECT_EQ(r2.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
    EXPECT_EQ(r2.winner, ControlSource::NONE);
    EXPECT_EQ(r2.cmd.pedal_ratio, 0);
    EXPECT_EQ(r2.cmd.brake_force, ActuatorCalibration{}.emergencyBrakeRaw());
    EXPECT_EQ(r2.cmd.steering, kNeutral);
}

// 故障注入：checksum 失败（数据被篡改/损坏）→ 该源不可信 → 安全降级。
TEST(CommandArbitration, CorruptedChecksumIgnored) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    auto bad = makeCandidate(ControlSource::PURE_PURSUIT, 95, 40, 0, 0.05);
    bad.checksum = static_cast<std::uint16_t>(bad.checksum + 1);  // 破坏校验和
    const SourceCandidate cands[] = {bad};
    auto r = arb.decide(false, true, std::span{cands}, cfg);
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
    EXPECT_EQ(r.cmd.pedal_ratio, 0);
}

// 故障注入：present=false（指令源崩溃/未发布）→ 安全降级。
TEST(CommandArbitration, MissingSourceIgnored) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    auto gone = makeCandidate(ControlSource::MPC, 88, 20, 0, 0.05);
    gone.present = false;
    const SourceCandidate cands[] = {gone};
    auto r = arb.decide(false, true, std::span{cands}, cfg);
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
}

// 故障注入：stop 锁存最高优先——即便有新鲜可信源，也必须输出安全停车，绝不放行。
TEST(CommandArbitration, StopActiveOverridesTrustedSource) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    auto pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 100, 0, 0.0);  // 满油门
    const SourceCandidate cands[] = {pp};
    auto r = arb.decide(/*stop_active=*/true, /*can_drive=*/true, std::span{cands}, cfg);
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::STOP_ACTIVE);
    EXPECT_EQ(r.cmd.pedal_ratio, 0);
    EXPECT_EQ(r.cmd.brake_force, ActuatorCalibration{}.emergencyBrakeRaw());
}

// 故障注入：任务态不允许行驶（未武装/IDLE）→ 安全制动而非执行控制源。
TEST(CommandArbitration, TaskNotDrivingProducesSafeBrake) {
    auto arb = makeArb();
    ArbitrationConfig cfg;
    auto pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 100, 0, 0.0);
    const SourceCandidate cands[] = {pp};
    auto r = arb.decide(/*stop_active=*/false, /*can_drive=*/false, std::span{cands}, cfg);
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::TASK_NOT_DRIVING);
    EXPECT_EQ(r.cmd.pedal_ratio, 0);
}

// 安全降级输出确定性：checksum 与指令同源、制动为锁定制动、无负值/溢出（对齐 ROS1 #6）。
TEST(CommandArbitration, SafeFallbackIsDeterministicAndValid) {
    auto arb = makeArb();
    std::span<const SourceCandidate> none{};
    auto r = arb.decide(true, true, none, ArbitrationConfig{});
    EXPECT_EQ(r.checksum, checksumRaw(r.cmd));  // 校验和与指令同源
    EXPECT_EQ(r.cmd.steering, kNeutral);
    EXPECT_LE(r.cmd.brake_force, 255);
    EXPECT_GE(r.cmd.brake_force, 1);
}

TEST(CommandArbitration, StateMachineDrivenDecision) {
    using common_msgs::vehicle::TaskSafetyStateMachine;
    auto arb = makeArb();
    ArbitrationConfig cfg;
    TaskSafetyStateMachine sm;
    sm.onArm();
    sm.onStart();
    auto pp = makeCandidate(ControlSource::PURE_PURSUIT, 95, 40, 0, 0.05);
    const SourceCandidate cands[] = {pp};
    // RUNNING + NORMAL → 控制
    EXPECT_EQ(arb.decide(sm, std::span{cands}, cfg).reason, ArbitrationReason::CONTROL);
    // 完赛 → 锁存停 → 即便有新鲜源也 STOP_ACTIVE
    sm.onFinish();
    auto r = arb.decide(sm, std::span{cands}, cfg);
    EXPECT_EQ(r.reason, ArbitrationReason::STOP_ACTIVE);
    EXPECT_TRUE(r.safe_fallback);
}
