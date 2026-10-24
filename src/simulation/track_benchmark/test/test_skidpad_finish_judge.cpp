// #90：八字绕环完赛判据（skidpad_finish_judge.hpp）——相位状态机 + 绕角累积计圈。
// 正样本：从 GenerateSkidpad() 中心线构造的合成轨迹（含 ≤0.3m 固定种子噪声）必须
// finished；负样本（仅单圆 / 仅一圈 / 反向绕行 / 跳过换圆直接出场）必须 not finished
// ——判据可证伪，不是「喂什么都 finished」。非法元数据（NaN / 非正半径 / 零圈数 /
// 圆心重合）必须安全：永不 finished 且不 UB（沿 #89 精神）。
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <random>
#include <utility>
#include <vector>

#include "track_benchmark/skidpad_finish_judge.hpp"
#include "track_benchmark/track_generator.hpp"

using namespace benchmark;

namespace {

// 判据 spec 只能从赛道元数据构造（沿 #81 finish_spec 的 runner 模式），测试同一入口。
SkidpadFinishSpec SpecFromTrack(const TrackDefinition& track) {
    SkidpadFinishSpec spec;
    if (track.hasSkidpad()) {
        spec.enabled = true;
        spec.right_center_x = track.skidpad->right_center_x;
        spec.right_center_y = track.skidpad->right_center_y;
        spec.left_center_x = track.skidpad->left_center_x;
        spec.left_center_y = track.skidpad->left_center_y;
        spec.path_radius = track.skidpad->path_radius;
        spec.required_laps = track.skidpad->required_laps;
        spec.exit_x = track.skidpad->exit_x;
    }
    return spec;
}

// 中心线分段索引（与 GenerateSkidpad() 的构造顺序一一对应）。
constexpr size_t kEntryEnd = 30;      // 进场 30 点（x=-15..-0.5）
constexpr size_t kRightEnd = 174;     // + 右圆 144 点（720°）
constexpr size_t kSwitchPoint = 175;  // + 换圆切点 (0,0) 的下一索引（切点在 174）
constexpr size_t kLeftEnd = 319;      // + 左圆 144 点
// 出场 30 点（x=0.5..15）到 349。

std::vector<std::pair<double, double>> PointsFrom(const std::vector<CenterlinePoint>& line, size_t begin, size_t end) {
    std::vector<std::pair<double, double>> pts;
    for (size_t i = begin; i < end && i < line.size(); ++i)
        pts.emplace_back(line[i].x, line[i].y);
    return pts;
}

void Feed(SkidpadFinishJudge& judge, const std::vector<std::pair<double, double>>& pts) {
    for (const auto& [x, y] : pts)
        judge.Update(x, y);
}

}  // namespace

// —— 正样本：生成器中心线即完赛轨迹 ——

TEST(SkidpadFinishJudgeTest, FullCenterlineFinishes) {
    const auto track = TrackGenerator::GenerateSkidpad();
    ASSERT_TRUE(track.hasSkidpad());
    SkidpadFinishJudge judge(SpecFromTrack(track));
    ASSERT_TRUE(judge.valid());

    // 相位按段推进：进场前段（远离圆环带）仍在 enter；右圆中途仍在 right_lap
    // （绕角 ~400° 未达门槛）；左圆后 exit；出场 finished。kTransition 是瞬态相位
    // ——门槛在右圆末段即越过，其后靠近切点的点立刻满足左圆带归属——不单独断言。
    Feed(judge, PointsFrom(track.centerline, 0, 20));
    EXPECT_EQ(judge.phase(), SkidpadFinishJudge::Phase::kEnter);
    Feed(judge, PointsFrom(track.centerline, 20, kEntryEnd + 80));
    EXPECT_EQ(judge.phase(), SkidpadFinishJudge::Phase::kRightLap);
    Feed(judge, PointsFrom(track.centerline, kEntryEnd + 80, kLeftEnd));
    EXPECT_EQ(judge.phase(), SkidpadFinishJudge::Phase::kExit);
    Feed(judge, PointsFrom(track.centerline, kLeftEnd, track.centerline.size()));
    EXPECT_TRUE(judge.IsFinished());
}

TEST(SkidpadFinishJudgeTest, UpdateReturnsTrueExactlyOnce) {
    // #91：finish_event_node 的「恰好一次」依赖 Update 只在首次完赛返回 true——
    // 完赛后的继续观测/状态抖动/乱序重放都不得二次触发。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    int fired = 0;
    for (const auto& [x, y] : PointsFrom(track.centerline, 0, track.centerline.size()))
        fired += judge.Update(x, y) ? 1 : 0;
    // 完赛后的重放（整条轨迹再喂一遍）也不得再触发。
    for (const auto& [x, y] : PointsFrom(track.centerline, 0, track.centerline.size()))
        fired += judge.Update(x, y) ? 1 : 0;
    EXPECT_TRUE(judge.IsFinished());
    EXPECT_EQ(fired, 1);
}

TEST(SkidpadFinishJudgeTest, NoisyCenterlineStillFinishes) {
    // ≤0.3m 横向噪声（固定种子）：绕角累积在整圈尺度上抵消扰动，计圈不得依赖位置采样。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    std::mt19937 rng(42);  // #75 精神：固定种子，噪声轨迹可复现
    std::uniform_real_distribution<double> noise(-0.3, 0.3);
    for (const auto& pt : track.centerline)
        judge.Update(pt.x + noise(rng), pt.y + noise(rng));
    EXPECT_TRUE(judge.IsFinished());
}

TEST(SkidpadFinishJudgeTest, WindingObservablesReachLapThreshold) {
    // 观测字段（#92 数据面门禁用）：双圆绕角都越过完赛门槛（2π·laps·fraction，
    // 状态机在门槛处即停止该圆累积），符号相反。门槛 10.05 rad ≫ 2π，足以支撑
    // 「真的在绕圈」类断言。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    Feed(judge, PointsFrom(track.centerline, 0, track.centerline.size()));
    const double threshold =
        2.0 * std::numbers::pi_v<double> * track.skidpad->required_laps * judge.spec().lap_complete_fraction;
    EXPECT_GE(judge.right_winding(), threshold);
    EXPECT_LE(judge.left_winding(), -threshold);
    EXPECT_GE(judge.right_winding(), 2.0 * std::numbers::pi_v<double>);  // 至少一整圈的铁证
}

// —— 负样本：四类退化轨迹必须 not finished（判据可证伪）——

TEST(SkidpadFinishJudgeTest, SingleCircleOnlyNeverFinishes) {
    // 仅绕第一圆两圈后停住：永远到不了 exit。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    Feed(judge, PointsFrom(track.centerline, 0, kSwitchPoint));
    EXPECT_FALSE(judge.IsFinished());
}

TEST(SkidpadFinishJudgeTest, MissingLapNeverFinishes) {
    // 第一圆少绕一圈、其余照走（换圆 + 左圆两圈 + 出场）：状态机卡在 right_lap，
    // 后续左圆绕行对第一圆圆心的净绕角为 0（圆心在另一圆外），不可能补足门槛。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    auto first = PointsFrom(track.centerline, 0, kEntryEnd + 72);  // 进场 + 右圆 360°（一圈）
    auto rest = PointsFrom(track.centerline, kRightEnd, track.centerline.size());
    Feed(judge, first);
    Feed(judge, rest);
    EXPECT_FALSE(judge.IsFinished());
    EXPECT_EQ(judge.phase(), SkidpadFinishJudge::Phase::kRightLap);
}

TEST(SkidpadFinishJudgeTest, ReversedCirclingNeverFinishes) {
    // 反向绕行第一圆（点序倒转 = 顺时针）：绕角符号错误，正向门槛永不达标。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    auto entry = PointsFrom(track.centerline, 0, kEntryEnd);
    auto reversed = PointsFrom(track.centerline, kEntryEnd, kRightEnd);
    std::reverse(reversed.begin(), reversed.end());
    auto rest = PointsFrom(track.centerline, kRightEnd, track.centerline.size());
    Feed(judge, entry);
    Feed(judge, reversed);
    Feed(judge, rest);
    EXPECT_FALSE(judge.IsFinished());
    EXPECT_LE(judge.right_winding(), 0.0) << "反向绕行的绕角必须为负（顺时针）";
}

TEST(SkidpadFinishJudgeTest, SkippingSecondCircleNeverFinishes) {
    // 绕满第一圆后直接出场（跳过换圆与第二圆）：可能误入 left_lap 相位，但出场通道
    // 对第二圆圆心的绕角量级 ≈ 1 rad ≪ 门槛，完赛不可能达成。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    Feed(judge, PointsFrom(track.centerline, 0, kSwitchPoint));
    Feed(judge, PointsFrom(track.centerline, kLeftEnd, track.centerline.size()));
    EXPECT_FALSE(judge.IsFinished());
}

// —— 非法元数据：永不 finished 且不 UB（沿 #89 退化输入防御）——

TEST(SkidpadFinishJudgeTest, InvalidSpecNeverFinishes) {
    const auto track = TrackGenerator::GenerateSkidpad();
    const auto pts = PointsFrom(track.centerline, 0, track.centerline.size());

    std::vector<SkidpadFinishSpec> bad_specs;
    SkidpadFinishSpec nan_center = SpecFromTrack(track);
    nan_center.right_center_x = std::numeric_limits<double>::quiet_NaN();
    bad_specs.push_back(nan_center);
    SkidpadFinishSpec zero_radius = SpecFromTrack(track);
    zero_radius.path_radius = 0.0;
    bad_specs.push_back(zero_radius);
    SkidpadFinishSpec zero_laps = SpecFromTrack(track);
    zero_laps.required_laps = 0;
    bad_specs.push_back(zero_laps);
    SkidpadFinishSpec coincident = SpecFromTrack(track);
    coincident.left_center_x = coincident.right_center_x;
    coincident.left_center_y = coincident.right_center_y;
    bad_specs.push_back(coincident);

    for (const auto& spec : bad_specs) {
        SkidpadFinishJudge judge(spec);
        EXPECT_FALSE(judge.valid());
        Feed(judge, pts);
        EXPECT_FALSE(judge.IsFinished());
    }

    // 未启用（非 skidpad 赛道）：同样安全。
    SkidpadFinishJudge disabled(SpecFromTrack(TrackGenerator::GenerateAcceleration()));
    EXPECT_FALSE(disabled.valid());
    Feed(disabled, pts);
    EXPECT_FALSE(disabled.IsFinished());
}

TEST(SkidpadFinishJudgeTest, NonFiniteSamplesAreIgnored) {
    // 观测流混入 NaN/Inf 拍：该拍被忽略，不污染绕角、不推进相位、不 UB。
    const auto track = TrackGenerator::GenerateSkidpad();
    SkidpadFinishJudge judge(SpecFromTrack(track));
    auto pts = PointsFrom(track.centerline, 0, track.centerline.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        if (i % 50 == 25) {
            judge.Update(std::numeric_limits<double>::quiet_NaN(), 0.0);
            judge.Update(0.0, std::numeric_limits<double>::infinity());
        }
        judge.Update(pts[i].first, pts[i].second);
    }
    EXPECT_TRUE(judge.IsFinished());
}
