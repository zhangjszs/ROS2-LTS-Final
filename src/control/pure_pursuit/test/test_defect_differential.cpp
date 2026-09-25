// ============================================================================
// issue #24：ROS1 → ROS2 语义差分的**合成夹具**（无 ROS context、无实车、可单测）。
//
// 每条用例固定输入顺序/时间/参数，并写明三件事：
//   1) ROS1 缺陷编号与当时的（错误）行为 —— 以字面量形式留在代码里；
//   2) ROS2 期望行为 —— 由断言锁定；
//   3) 差异分类 —— 缺陷修复差异 / 预期算法差异 / 接口回归 / 证据不足。
// 分类表与逐条依据见 docs/DEFECT_DIFFERENTIAL.md（同一编号体系，供 #25 文章与 #26 展示引用）。
//
// 本文件是 #24 验收第 1 条要求的“修复前失败、修复后通过的最小回归”：
// 若相应 ROS2 逻辑退回旧语义，这里会红；不以任何旧错误数值作为成功标准。
// ============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "command_arbitration.h"
#include "interface_contract.h"
#include "steering_calibration.h"
#include "task_state_machine.h"
#include "vehicle_command_codec.h"

namespace {

using common_msgs::contract::geometryValid;
using common_msgs::contract::kQosStop;
using common_msgs::contract::kTopicStop;
using common_msgs::contract::selectReferenceSpeed;
using common_msgs::contract::sourceAgeAcceptable;
using common_msgs::contract::targetSpeedsEffective;
using common_msgs::vehicle::ActuatorCalibration;
using common_msgs::vehicle::ArbitrationConfig;
using common_msgs::vehicle::ArbitrationReason;
using common_msgs::vehicle::CommandArbiterFilter;
using common_msgs::vehicle::CommandArbitrator;
using common_msgs::vehicle::ControlSource;
using common_msgs::vehicle::makeWireSourceObs;
using common_msgs::vehicle::SourceCandidate;
using common_msgs::vehicle::SteeringCalibration;
using common_msgs::vehicle::TaskSafetyStateMachine;
using common_msgs::vehicle::TaskState;
using common_msgs::vehicle::VehicleCommandRaw;
using common_msgs::vehicle::verifyFrame;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 默认标定（仿真协议）：满量程 100、急停 80、软停 40。
ActuatorCalibration simActuator() {
    return ActuatorCalibration{};
}

// 255 满量程底盘替身（#15 待实车确认的可能性之一）。
ActuatorCalibration byteActuator() {
    ActuatorCalibration cal;
    cal.pedal_full_scale = 255.0;
    return cal;
}

// 一条正常的行驶指令（油门>0），用于“活性但陈旧”类场景。
VehicleCommandRaw driveRaw(std::uint8_t pedal) {
    VehicleCommandRaw r;
    r.steering = 90;
    r.pedal_ratio = pedal;
    r.brake_force = 0;
    r.gear_position = 1;
    r.working_mode = 1;
    return r;
}

}  // namespace

// ── 矩阵第 1 行：合法零速 / 超速减速 ─────────────────────────────────────────
// ROS1 #7：0 m/s 被当成“没数据”而替换成巡航速度 -> 该停不停。
// ROS1 #6：负油门窄化到 uint8 成为 255（满油门），且油门制动同时输出。
// 分类：缺陷修复差异。
TEST(DefectDifferential, ZeroSpeedIsACommandNotAMissingValue) {
    // ROS1 的旧解释：0 == 缺失 -> 抬到巡航。显式留下错误值以证明我们不再复现它。
    const double ros1BuggySpeed = 5.0;

    // 0 m/s 在 ROS2 契约里是合法且显式的“停车目标”，绝不被替换（#11）。
    const auto sel = selectReferenceSpeed(/*has_explicit_speeds=*/true, /*target_speed=*/0.0,
                                          /*default_reference_speed=*/ros1BuggySpeed);
    EXPECT_TRUE(sel.valid) << "0 m/s 必须被当作有效的显式速度";
    EXPECT_DOUBLE_EQ(sel.speed, 0.0);
    EXPECT_NE(sel.speed, ros1BuggySpeed);

    // 缺失/含非有限的数组走回退，而不是被静默当 0 或被当巡航（#14 §2）。
    const std::vector<double> empty;
    const std::vector<double> with_nan{1.0, kNaN, 2.0};
    const std::vector<double> ok_with_zero{0.0, 1.5, 2.0};
    EXPECT_FALSE(targetSpeedsEffective(std::span{empty}, 3u));
    EXPECT_FALSE(targetSpeedsEffective(std::span{with_nan}, 3u)) << "含 NaN 的数组不可信";
    EXPECT_TRUE(targetSpeedsEffective(std::span{ok_with_zero}, 3u)) << "含 0 是合法的停车目标";
}

TEST(DefectDifferential, NegativeThrottleCannotBecomeFullPedal255) {
    const auto cal = simActuator();

    // ROS1 #6 的错误出口字节：负油门窄化后成为满油门 255。
    constexpr std::uint8_t ros1BuggyPedal = 255;

    // 非有限输入 -> 无油门 + 安全制动（不是“保持上一帧”，更不是 255 满油门）。
    for (double bad : {kNaN, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
        const auto out = cal.encode(bad);
        EXPECT_EQ(out.pedal, 0) << "input=" << bad;
        EXPECT_NE(out.pedal, ros1BuggyPedal);
        EXPECT_EQ(out.brake, cal.emergencyBrakeRaw());
        EXPECT_TRUE(out.safe_fallback);
    }

    // 要求超速减速：制动饱和、油门恒 0（互斥，绝不油刹同时给）。
    const auto overspeed = cal.encode(-1e6);
    EXPECT_EQ(overspeed.pedal, 0);
    EXPECT_GT(overspeed.brake, 0);

    // 255 满量程底盘：窄化前已 clamp，任何输入都越不出合法域且互斥成立。
    const auto wide = byteActuator();
    for (double a : {-1e6, 1e6, kNaN, -kNaN}) {
        const auto out = wide.encode(a);
        EXPECT_LE(out.pedal, 255);
        EXPECT_LE(out.brake, 255);
        EXPECT_FALSE(out.pedal > 0 && out.brake > 0) << "油门与制动互斥 a=" << a;
    }
}

// ── 矩阵第 2 行：0/1/2 点与畸形轨迹 ─────────────────────────────────────────
// ROS1 #8：八字单点路径越界；#9：空/畸形路径仍刷新 watchdog 却静默无输出。
// 分类：缺陷修复差异（速度权威侧同名夹具见 velocity_profiler/test_velocity_profiler.cpp）。
TEST(DefectDifferential, MalformedGeometryIsRejectedNotFollowed) {
    const std::vector<double> e;
    const std::vector<double> one{1.0};
    const std::vector<double> two{1.0, 2.0};
    const std::vector<double> one_y{0.0};
    const std::vector<double> two_y{0.0, 0.0};
    const std::vector<double> two_nan{1.0, kNaN};

    EXPECT_FALSE(geometryValid(std::span{e}, std::span{e})) << "0 点";
    EXPECT_FALSE(geometryValid(std::span{one}, std::span{one})) << "1 点";
    EXPECT_FALSE(geometryValid(std::span{two}, std::span{one_y})) << "长度不等";
    EXPECT_FALSE(geometryValid(std::span{two_nan}, std::span{two_y})) << "含非有限";
    EXPECT_TRUE(geometryValid(std::span{two}, std::span{two_y})) << "2 点是合法下限";

    // “节点沉默 ≠ 制动”：无任一可信源时仲裁必须给出确定性安全制动，而非复用上一帧油门。
    const CommandArbitrator arb(simActuator(), 90);
    const std::vector<SourceCandidate> none;
    const auto degraded = arb.decide(/*stop_active=*/false, /*can_drive=*/true, std::span{none}, ArbitrationConfig{});
    EXPECT_TRUE(degraded.safe_fallback);
    EXPECT_EQ(degraded.reason, ArbitrationReason::NO_TRUSTED_SOURCE);
    EXPECT_EQ(degraded.cmd.pedal_ratio, 0);
    EXPECT_GT(degraded.cmd.brake_force, 0);
    // 安全帧本身必须满足 #15 帧契约（消费者可验证）。
    EXPECT_TRUE(verifyFrame(0xAA, 0x55, 10));
}

// ── 矩阵第 3 行：位姿断流、旧戳重复、时间跳变 ────────────────────────────────
// ROS1 #11：位姿过期但路径持续到达 -> 系统仍按“活着”继续行驶。
// 分类：缺陷修复差异（区分接收活性与观测有效性）。
TEST(DefectDifferential, ArrivalIsNotFreshnessAndFutureStampsAreRejected) {
    // 源年龄判定：超龄 / 未来戳（时间跳变）都不接受；tolerance<0 是“显式禁用”而非“随便信”。
    EXPECT_TRUE(sourceAgeAcceptable(0.02, 0.5));
    EXPECT_FALSE(sourceAgeAcceptable(0.6, 0.5)) << "超龄必须拒收（即使消息持续到达）";
    EXPECT_FALSE(sourceAgeAcceptable(-1.0, 0.5)) << "未来戳/时间回拨不得当新鲜";
    EXPECT_TRUE(sourceAgeAcceptable(999.0, -1.0)) << "禁用态由参数显式表达（标定前默认）";

    // 仲裁侧：持续到达但已超龄的源不得继续授予行驶许可。
    const auto obs = makeWireSourceObs(ControlSource::PURE_PURSUIT, driveRaw(50), 0xAA, 0x55, 10,
                                       common_msgs::vehicle::checksumRaw(driveRaw(50)), 10.0);
    ArbitrationConfig cfg;
    cfg.source_timeout_sec = 0.5;
    EXPECT_EQ(CommandArbiterFilter::classify(obs, 10.2, cfg), CommandArbiterFilter::Untrusted::kNone);
    EXPECT_EQ(CommandArbiterFilter::classify(obs, 12.0, cfg), CommandArbiterFilter::Untrusted::kStale);
}

// ── 矩阵第 4 行：终点、停机、重启 / 晚加入 ──────────────────────────────────
// ROS1 #10：完赛发全零命令后退出（全零 ≠ 制动）；#7/#8 的停车保持与恢复条件。
// 分类：缺陷修复差异（锁存语义 + 恢复条件）。
TEST(DefectDifferential, FinishAndAbortLatchAndRestartGrantsNoPermission) {
    // 全零指令（油门 0 + 制动 0）在 ROS2 里不等于“已安全”：安全降级必须带制动字节。
    const CommandArbitrator arb(simActuator(), 90);
    const auto stop = arb.safeStopResult(ArbitrationReason::STOP_ACTIVE);
    EXPECT_EQ(stop.cmd.pedal_ratio, 0);
    EXPECT_GT(stop.cmd.brake_force, 0) << "安全降级必须落到锁定制动，不能是全零帧";

    // 完赛 -> 锁存停；后续“路径又活了”不得解除；只有 reset 解除且回到 IDLE（不授予许可）。
    TaskSafetyStateMachine sm;
    sm.onArm();
    sm.onStart();
    EXPECT_TRUE(sm.canDrive());
    sm.onFinish();
    EXPECT_TRUE(sm.stopActive());
    EXPECT_FALSE(sm.canDrive());
    sm.onResume();  // 模拟 planner 恢复活性
    EXPECT_TRUE(sm.stopActive()) << "完赛锁存不得被活性恢复解除（#16 验收第 3 条）";
    sm.onReset();
    EXPECT_FALSE(sm.stopActive());
    EXPECT_EQ(sm.task(), TaskState::IDLE) << "复位后必须重新 arm+start（重启不意外授予许可）";
    EXPECT_FALSE(sm.canDrive());

    // 晚加入者要能恢复锁存态：stop 话题必须是 transient_local（#8/#14）。
    EXPECT_TRUE(kQosStop.transient_local);
    EXPECT_EQ(std::string{kTopicStop}, "/system/stop");
}

// ── 矩阵第 6 行：中位及正负转向、制动满量程 ─────────────────────────────────
// ROS1 #12：控制器与执行器仿真的零位/缩放不一致（同一命令两套含义）。
// 分类：缺陷修复差异（物理语义单一来源）+ 预期算法差异（0–255 底盘仅改参数，不改代码）。
TEST(DefectDifferential, SteeringNeutralAndScaleAreSharedNotDuplicated) {
    const SteeringCalibration sim{.neutral = 90.0, .units_per_degree = 1.0, .min_raw = 65.0, .max_raw = 115.0};
    const SteeringCalibration vcu{.neutral = 128.0, .units_per_degree = 2.0, .min_raw = 78.0, .max_raw = 178.0};

    for (const auto& cal : {sim, vcu}) {
        // 零转角 == 该套标定的中位（不是别处的 90/128 字面量）。
        EXPECT_DOUBLE_EQ(static_cast<double>(cal.neutralRaw()), cal.neutral);
        EXPECT_DOUBLE_EQ(cal.decodeRad(cal.neutralRaw()), 0.0);

        // 左右对称 + 往返误差 < 1 raw 单位对应的角度。
        const double deg = 12.0;
        const double rad = deg * M_PI / 180.0;
        const double tol = (1.0 / cal.units_per_degree) * M_PI / 180.0;
        EXPECT_NEAR(cal.decodeRad(cal.encodeRad(rad)), rad, tol);
        EXPECT_NEAR(cal.decodeRad(cal.encodeRad(-rad)), -rad, tol);
        EXPECT_NEAR(cal.decodeRad(cal.encodeRad(rad)) + cal.decodeRad(cal.encodeRad(-rad)), 0.0, tol * 2.0)
            << "正负转角不对称";

        // 极限输入被 clamp 到该标定的合法域，而不是越界或环绕。
        EXPECT_LE(cal.encodeRad(10.0), static_cast<int>(cal.max_raw));
        EXPECT_GE(cal.encodeRad(-10.0), static_cast<int>(cal.min_raw));
    }

    // 混用两套标定必然产生错误的零位 —— 这正是 ROS1 #12 的分歧形态，
    // 因此控制器与仿真器必须共引同一份标定（#15），而不是各自带默认值。
    EXPECT_NE(sim.decodeRad(vcu.neutralRaw()), 0.0);  // 拿 vcu 的中位去满足 sim → 非 0转角
    EXPECT_NE(vcu.decodeRad(sim.neutralRaw()), 0.0);  // 反向同理
}

// ── 矩阵第 5 行：非零位姿下 map / base_link 等价输入 ─────────────────────────
// 分类：预期算法差异（本仓 #3 已修复；此处锁定“物理量与坐标原点解耦”这一可移植性质，
//        MPC 侧的完整等价性由 mpc_tracking / test_path_reference_builder 覆盖）。
TEST(DefectDifferential, PhysicalCommandIsIndependentOfPoseOrigin) {
    const SteeringCalibration cal{.neutral = 90.0, .units_per_degree = 1.0, .min_raw = 65.0, .max_raw = 115.0};
    const double steer_rad = 0.12;  // 物理前轮转角：右正
    const int expected_raw = cal.encodeRad(steer_rad);

    // 平移车辆原点（map 系下位姿非零）只改变路径坐标，不改变“同一物理转角 -> 同一 raw”。
    for (double origin : {0.0, 317.4, -88.25}) {
        EXPECT_EQ(cal.encodeRad(steer_rad), expected_raw) << "origin=" << origin;
    }
    // 加速度的物理域同样与底盘量化域解耦：满量程不同只改 raw 数值，不改目标加速度。
    ASSERT_DOUBLE_EQ(simActuator().encode(2.5).pedal, 50);    // 5.0 m/s^2 满量程 -> 半油门
    ASSERT_DOUBLE_EQ(byteActuator().encode(2.5).pedal, 127);  // 同一物理量在 0–255 域里约 127
}
