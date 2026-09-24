// issue #16：跨帧有状态仲裁滤波器 —— 去抖/切换驻留/stale/stop 复位的确定性回归。
// 用虚拟单调时钟逐帧喂入，无需 ROS/实车即可判定通过/失败（CI 门禁）。
#include <gtest/gtest.h>

#include <array>
#include <span>

#include "command_arbitration.h"

using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::ArbitrationConfig;
using common_msgs::vehicle::ArbitrationReason;
using common_msgs::vehicle::checksumRaw;
using common_msgs::vehicle::CommandArbiterFilter;
using common_msgs::vehicle::CommandArbitrator;
using common_msgs::vehicle::ControlSource;

namespace {
constexpr std::uint8_t kNeutral = 90;

CommandArbiterFilter makeFilter() {
    return CommandArbiterFilter(CommandArbitrator(ActuatorCalibration{}, kNeutral));
}

CommandArbiterFilter::SourceObs makeObs(ControlSource src, std::uint8_t pedal, double last_rx, bool present = true) {
    CommandArbiterFilter::SourceObs o;
    o.source = src;
    o.cmd.steering = kNeutral;
    o.cmd.pedal_ratio = pedal;
    o.cmd.brake_force = 0;
    o.cmd.gear_position = 1;
    o.cmd.working_mode = 1;
    o.cmd.racing_num = 7;
    o.checksum = checksumRaw(o.cmd);
    o.last_rx_sec = last_rx;
    o.present = present;
    return o;
}
}  // namespace

TEST(ArbiterFilter, FreshPreferredWinsThenSticky) {
    auto f = makeFilter();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::PURE_PURSUIT;
    cfg.switch_dwell_sec = 0.0;
    auto o1 = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 1.0)};
    auto r1 = f.update(1.0, false, true, std::span{o1}, cfg);
    EXPECT_EQ(r1.winner, ControlSource::PURE_PURSUIT);
    EXPECT_FALSE(r1.safe_fallback);
    auto o2 = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 1.02)};
    auto r2 = f.update(1.02, false, true, std::span{o2}, cfg);
    EXPECT_EQ(r2.winner, ControlSource::PURE_PURSUIT);  // sticky
    EXPECT_EQ(f.winner(), ControlSource::PURE_PURSUIT);
}

TEST(ArbiterFilter, PreferredUsedWhenBothValid) {
    auto f = makeFilter();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::MPC;
    auto obs = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 2.0), makeObs(ControlSource::MPC, 20, 2.0)};
    auto r = f.update(2.0, false, true, std::span{obs}, cfg);
    EXPECT_EQ(r.winner, ControlSource::MPC);
    EXPECT_EQ(r.cmd.pedal_ratio, 20);
}

// 去抖：新源须连续可信 >= dwell 才接管；PP 掉线瞬间不立即切到刚出现的 MPC。
TEST(ArbiterFilter, DwellPreventsInstantTakeover) {
    auto f = makeFilter();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::PURE_PURSUIT;
    cfg.source_timeout_sec = 0.5;
    cfg.switch_dwell_sec = 1.0;
    // t=10：PP 建立并胜出
    auto o0 = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 10.0)};
    auto r0 = f.update(10.0, false, true, std::span{o0}, cfg);
    EXPECT_EQ(r0.winner, ControlSource::PURE_PURSUIT);
    // t=10.8：PP last_rx 仍 10.0（age 0.8>0.5）→ 失效；MPC 刚出现（连续可信 0 < dwell）→ 降级不接管
    auto s1 = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 10.0), makeObs(ControlSource::MPC, 20, 10.8)};
    auto r1 = f.update(10.8, false, true, std::span{s1}, cfg);
    EXPECT_TRUE(r1.safe_fallback);
    EXPECT_EQ(r1.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
    EXPECT_EQ(r1.cmd.pedal_ratio, 0);
    EXPECT_EQ(r1.cmd.brake_force, ActuatorCalibration{}.emergencyBrakeRaw());
    // t=11.9：MPC 连续可信 1.1s >= dwell → 接管
    auto s2 = std::array{makeObs(ControlSource::MPC, 20, 11.9)};
    auto r2 = f.update(11.9, false, true, std::span{s2}, cfg);
    EXPECT_EQ(r2.winner, ControlSource::MPC);
    EXPECT_FALSE(r2.safe_fallback);
}

// stale：期望源超时 → 回退到另一可信源（dwell=0）。
TEST(ArbiterFilter, StalePreferredFallsBackToOther) {
    auto f = makeFilter();
    ArbitrationConfig cfg;
    cfg.preferred = ControlSource::PURE_PURSUIT;
    cfg.source_timeout_sec = 0.5;
    auto obs = std::array{makeObs(ControlSource::PURE_PURSUIT, 40, 4.0),  // 陈旧
                          makeObs(ControlSource::MPC, 20, 5.0)};          // 新鲜
    auto r = f.update(5.0, false, true, std::span{obs}, cfg);
    EXPECT_EQ(r.winner, ControlSource::MPC);
    EXPECT_FALSE(r.safe_fallback);
}

// stop 锁存：即便有新鲜可信源也强制安全制动，并清空 winner；解除后可重新接管。
TEST(ArbiterFilter, StopResetsWinnerAndForcesBrake) {
    auto f = makeFilter();
    ArbitrationConfig cfg;
    auto establish = std::array{makeObs(ControlSource::PURE_PURSUIT, 100, 1.0)};
    f.update(1.0, false, true, std::span{establish}, cfg);
    EXPECT_EQ(f.winner(), ControlSource::PURE_PURSUIT);
    auto stopped = std::array{makeObs(ControlSource::PURE_PURSUIT, 100, 1.02)};
    auto r = f.update(1.02, /*stop_active=*/true, false, std::span{stopped}, cfg);
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::STOP_ACTIVE);
    EXPECT_EQ(f.winner(), ControlSource::NONE);  // 已复位
    // 解除 stop 后重新接管（dwell=0）
    auto recovered = std::array{makeObs(ControlSource::PURE_PURSUIT, 100, 1.04)};
    auto r2 = f.update(1.04, false, true, std::span{recovered}, cfg);
    EXPECT_EQ(r2.winner, ControlSource::PURE_PURSUIT);
}

// checksum 篡改 → 该源不可信 → 无合格源降级（跨帧）。
TEST(ArbiterFilter, CorruptedSourceDegrades) {
    auto f = makeFilter();
    auto bad = makeObs(ControlSource::MPC, 20, 3.0);
    bad.checksum = static_cast<std::uint16_t>(bad.checksum + 1);
    auto obs = std::array{bad};
    auto r = f.update(3.0, false, true, std::span{obs}, ArbitrationConfig{});
    EXPECT_TRUE(r.safe_fallback);
    EXPECT_EQ(r.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
}
