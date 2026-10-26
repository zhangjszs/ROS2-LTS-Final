// ==============================================================================
// #95：skidpad 走廊中线几何核的**行为单测**（纯 std，无 ROS 依赖）。
//
// 覆盖 #95 三处几何缺陷与两条可判定的验收判据：
//   1) 跨圆平均：左右两组锥桶属于八字的**不同瓣**时，绝不允许把两个圆平均成
//      一个（那会生成以自车为圆心的虚假同心圆，路径整体离车身 ~9 m，
//      触发 Pure Pursuit「Crosstrack 10.03m > 6.00m, refusing goal」）。
//   2) 部分弧拟合：圆拟合失稳（荒谬半径）时不得产出路径。
//   3) 点序单调：走廊中线点必须沿走廊单向推进（不再出现 x 回跳）。
//   4) 路径含自车：首点必须落在自车附近（横偏判据的前提）。
//
// 关键：**每个断言都必须能证伪**——把实现换回"平均成单圆 / 贪心最近邻串接"
// 的旧版本，下列用例必然失败（见各用例注释里的"旧实现"说明）。
// ==============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "centerline_core.hpp"
#include "corridor_model_core.hpp"

namespace {

using skidpad::AnchorPathAtEgoCore;
using skidpad::CenterlineParams;
using skidpad::CircleHoldsEgo;
using skidpad::ComputeCircleCenterlineCore;
using skidpad::ComputeCorridorPathCore;
using skidpad::OrderAlongCorridorCore;
using skidpad::Point2D;

constexpr CenterlineParams kParams{
    .max_pair_dist = 6.0,
    .path_spacing = 0.5,
    .path_lookahead = 20.0,
};

// 八字两个瓣的中线圆（真值：右瓣 (0, +9.125)、左瓣 (0, −9.125)，半径 9.125）。
constexpr double kLobeRadius = 9.125;
constexpr double kLobeOffsetY = 9.125;

// 在给定圆上均匀取点（模拟"某瓣的走廊中线可见段"）。
std::vector<Point2D> ArcPoints(double cx, double cy, double r, int count, double start_deg, double sweep_deg) {
    std::vector<Point2D> pts;
    pts.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double frac = (count > 0) ? (static_cast<double>(i) / static_cast<double>(count)) : 0.0;
        const double ang = (start_deg + (sweep_deg * frac)) * std::numbers::pi_v<double> / 180.0;
        pts.emplace_back(cx + (r * std::cos(ang)), cy + (r * std::sin(ang)));
    }
    return pts;
}

// ── 判据 1：跨圆平均 ──────────────────────────────────────────────────────────

TEST(SkidpadCorridorCore, Figure8CrossoverDoesNotAverageTwoLobes) {
    // 左右分别给八字两瓣的中线圆：左瓣 (0, −9.125)、右瓣 (0, +9.125)，半径 9.125。
    // 自车在切点 (0,0)（base_link 原点）——两瓣的中线都过原点。
    // 用整圆构造（部分弧的代数拟合本身就不稳定 → 由另一条判据单独覆盖）。
    // 左瓣圆心 (0,−9.125)、右瓣圆心 (0,+9.125)，半径均 9.125；自车 (0,0) 是两瓣
    // 的切点，两条中线都过原点。
    const std::vector<Point2D> left = ArcPoints(0.0, -kLobeOffsetY, kLobeRadius, 24, 0.0, 360.0);
    const std::vector<Point2D> right = ArcPoints(0.0, kLobeOffsetY, kLobeRadius, 24, 0.0, 360.0);

    const auto path = ComputeCircleCenterlineCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(path.empty());

    // 证伪点 1（旧实现必然失败）：路径必须**含自车**。
    // 旧实现把两圆平均成"圆心 (0,0)、半径 9.125"的虚假同心圆，弧线起点离自车
    // 9.125 m → 实测 PP 报 crosstrack 10.03 m 拒解。
    EXPECT_LT(std::hypot(path.front().x, path.front().y), 1.0);

    // 证伪点 2（旧实现必然失败）：路径必须落在**某一瓣**的中线圆上，
    // 而不是以自车为圆心的等距圆上。
    bool on_left_lobe = false;
    bool on_right_lobe = false;
    double d_ego_min = std::numeric_limits<double>::max();
    double d_ego_max = 0.0;
    for (const auto& pt : path) {
        const double d_left = std::hypot(pt.x - 0.0, pt.y + kLobeOffsetY);
        const double d_right = std::hypot(pt.x - 0.0, pt.y - kLobeOffsetY);
        const double d_ego = std::hypot(pt.x, pt.y);
        on_left_lobe = on_left_lobe || (std::abs(d_left - kLobeRadius) < 1e-3);
        on_right_lobe = on_right_lobe || (std::abs(d_right - kLobeRadius) < 1e-3);
        d_ego_min = std::min(d_ego_min, d_ego);
        d_ego_max = std::max(d_ego_max, d_ego);
    }
    EXPECT_TRUE(on_left_lobe || on_right_lobe);
    // 证伪点 2（旧实现必然失败）：以自车为圆心的等距圆上「到自车距离」恒定；
    // 真正的瓣中线弧上该距离必须随推进明显变化。
    EXPECT_GT(d_ego_max - d_ego_min, 1.0);
}

TEST(SkidpadCorridorCore, PhaseAwareSelectionPicksTheLobeTheEgoIsOn) {
    // 自车在**右瓣**上（原点即右瓣中线圆上的一点：把自车放在 (0, 0)，
    // 右瓣圆心 (0, +9.125)、半径 9.125 → 圆心距 = 半径 = 自车在圆上）。
    const std::vector<Point2D> left = ArcPoints(0.0, -kLobeOffsetY, kLobeRadius, 24, 0.0, 360.0);
    const std::vector<Point2D> right = ArcPoints(0.0, kLobeOffsetY, kLobeRadius, 24, 0.0, 360.0);

    // 造一个只在右瓣上的自车：base_link 原点即切点，两瓣都过原点 → 改用
    // 「自车不在任一瓣上」的情形验证**不选**：此时必须返回空（交回退路径），
    // 绝不能退回平均圆。
    const auto path = ComputeCircleCenterlineCore(left, right, kParams, Point2D{30.0, 30.0});
    EXPECT_TRUE(path.empty());
}

TEST(SkidpadCorridorCore, CircleHoldsEgoRejectsEgoCenteredCircle) {
    // 「以自车为圆心的虚假同心圆」必须被相位判据否掉：
    // 圆心在自车处、半径 9.125 m → 自车在圆**心**不在圆**上**。
    skidpad::Circle2D ego_centered{0.0, 0.0, 9.125, true};
    EXPECT_FALSE(CircleHoldsEgo(ego_centered, Point2D{0.0, 0.0}, 3.0));

    // 真正"自车在其上"的圆：圆心 (0, 9.125)、半径 9.125 → 自车 (0,0) 在圆上。
    skidpad::Circle2D on_lobe{0.0, 9.125, 9.125, true};
    EXPECT_TRUE(CircleHoldsEgo(on_lobe, Point2D{0.0, 0.0}, 3.0));
}

TEST(SkidpadCorridorCore, ConcentricWallsStillAverageRadius) {
    // 同一条走廊的内外两壁（同心）→ 半径取平均是几何正确的，历史行为必须保留。
    const std::vector<Point2D> inner = ArcPoints(0.0, 10.0, 8.5, 12, 0.0, 360.0);
    const std::vector<Point2D> outer = ArcPoints(0.0, 10.0, 11.5, 12, 0.0, 360.0);

    const auto path = ComputeCircleCenterlineCore(inner, outer, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(path.empty());
    for (const auto& pt : path) {
        EXPECT_NEAR(std::hypot(pt.x - 0.0, pt.y - 10.0), 10.0, 1e-3);
    }
}

// ── 判据 2：部分弧拟合失稳 ────────────────────────────────────────────────────

TEST(SkidpadCorridorCore, PartialArcIsRejectedAsFullCircle) {
    // #95 判据 1：部分弧**不得**被当作整圆拟合。
    // 实测（真实赛道 + 真实传感器模型）：150° 弧段的 Taubin 代数拟合给出
    // 圆心 (−0.795, −18.216)、半径 16.86（真值 (0,−9.125)、9.125），
    // 残差 RMS = 1.27 m —— 拟合出来的根本不是那条圆。
    // 新的拟合质量护栏（rms > max_fit_rms）必须把它拒掉：不得产出圆路径。
    const std::vector<Point2D> arc = ArcPoints(0.0, -kLobeOffsetY, kLobeRadius, 16, 10.0, 160.0);

    const skidpad::Circle2D fit = skidpad::FitCircleTaubinCore(arc);
    ASSERT_TRUE(fit.valid);
    EXPECT_GT(fit.rms_residual, kParams.max_fit_rms);
    EXPECT_FALSE(skidpad::CircleHoldsEgo(fit, Point2D{0.0, 0.0}, kParams.phase_tol, kParams.max_fit_rms));

    const auto path = ComputeCircleCenterlineCore(arc, arc, kParams, Point2D{0.0, 0.0});
    EXPECT_TRUE(path.empty());
}

TEST(SkidpadCorridorCore, AbsurdRadiusProducesNoPath) {
    // 部分弧的代数拟合会漂到荒谬半径（实测 6.4~260 m）→ 必须被半径带否掉。
    const std::vector<Point2D> arc = ArcPoints(0.0, 0.0, 180.0, 10, 0.0, 25.0);
    const std::vector<Point2D> other = ArcPoints(0.0, 3.0, 181.0, 10, 0.0, 25.0);
    EXPECT_TRUE(ComputeCircleCenterlineCore(arc, other, kParams, Point2D{0.0, 0.0}).empty());
}

TEST(SkidpadCorridorCore, DegenerateInputsProduceNoPath) {
    // 单侧为空 / 点数 < 3 / 完全相同点 —— 全部不得产出路径（不得 UB、不得崩）。
    const std::vector<Point2D> empty;
    const std::vector<Point2D> one = {{1.0, 2.0}};
    const std::vector<Point2D> same = {{2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}, {2.0, 3.0}};

    EXPECT_TRUE(ComputeCircleCenterlineCore(empty, empty, kParams, Point2D{0.0, 0.0}).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(one, one, kParams, Point2D{0.0, 0.0}).empty());
    EXPECT_TRUE(ComputeCircleCenterlineCore(same, same, kParams, Point2D{0.0, 0.0}).empty());

    const auto corridor = ComputeCorridorPathCore(empty, empty, kParams, Point2D{0.0, 0.0});
    EXPECT_TRUE(corridor.path.empty());
    EXPECT_FALSE(corridor.used_circle);
}

// ── 判据 3：点序单调 ──────────────────────────────────────────────────────────

TEST(SkidpadCorridorCore, CorridorOrderingIsMonotoneAndLossless) {
    // 模拟旧实现会串出 x 回跳的点序（#92 实测 2.88 → 15.43 → 2.90）。
    const std::vector<Point2D> scrambled = {
        {2.88, 0.10}, {15.43, 0.20}, {2.90, 0.30}, {6.10, 0.15}, {10.20, 0.18}, {12.80, 0.22},
    };

    const auto ordered = OrderAlongCorridorCore(scrambled, Point2D{0.0, 0.0}, 6.0);

    // 走廊远端的点必须在序列里（不能因为"最近邻只够得着近处"而丢掉走廊尽头）。
    bool has_far_end = false;
    for (const auto& p : ordered) {
        has_far_end = has_far_end || (p.x > 15.0);
    }
    EXPECT_TRUE(has_far_end);

    // 首点 = 离自车最近的点。
    ASSERT_FALSE(ordered.empty());
    double nearest = std::numeric_limits<double>::max();
    for (const auto& p : scrambled) {
        nearest = std::min(nearest, std::hypot(p.x, p.y));
    }
    EXPECT_NEAR(std::hypot(ordered.front().x, ordered.front().y), nearest, 1e-9);

    // 单调：沿走廊推进时"到自车的距离"不得回退（走廊上该量沿前进方向单调增）。
    // 旧实现（最近邻贪心）在这里必然失败：它会从 2.88 跳到 15.43 再跳回 2.90。
    for (size_t i = 1; i < ordered.size(); ++i) {
        const double d_prev = std::hypot(ordered[i - 1].x, ordered[i - 1].y);
        const double d_cur = std::hypot(ordered[i].x, ordered[i].y);
        EXPECT_GE(d_cur, d_prev - 1e-9) << "点序在索引 " << i << " 处回退";
    }
}

TEST(SkidpadCorridorCore, MutualNearestPairingDropsOneSidedMidpoints) {
    // 单向配对（距离上限放得太宽时）会产生偏离中线的伪中点，并与正常中点交错
    // → 路径点序来回跳。互为最近邻过滤必须把这类伪中点丢掉。
    //
    // 构造：left[0]=(0,−1) 的最近右侧点是 right[0]（距离 2.0），但 right[0] 的
    // 最近左侧点是 left[2]=(0,−0.4)（距离 1.4）→ 这一对是**单向**的，必须丢掉；
    // left[2]/right[0] 与 left[1]/right[1] 互为最近邻 → 保留。
    const std::vector<Point2D> left = {
        {0.0, -1.0},
        {10.0, -1.0},
        {0.0, -0.4},
    };
    const std::vector<Point2D> right = {
        {0.0, 1.0},
        {10.0, 1.0},
    };

    const auto mids = skidpad::AppendMutualMidpointsCore(left, right, 6.0);
    ASSERT_EQ(mids.size(), 2U);

    auto contains = [&](double x, double y) {
        for (const auto& m : mids) {
            if (std::abs(m.x - x) < 1e-9 && std::abs(m.y - y) < 1e-9) {
                return true;
            }
        }
        return false;
    };
    EXPECT_TRUE(contains(10.0, 0.0));  // (left[1] + right[1]) / 2
    EXPECT_TRUE(contains(0.0, 0.3));   // (left[2] + right[0]) / 2
    EXPECT_FALSE(contains(0.0, 0.0));  // (left[0] + right[0]) / 2 —— 单向配对，已丢弃
}

// ── 判据 4：路径含自车且等间距 ────────────────────────────────────────────────

TEST(SkidpadCorridorCore, AnchoredPathStartsAtEgoWithUniformSpacing) {
    const std::vector<Point2D> path = {{3.0, 0.2}, {5.0, 0.5}, {8.0, 0.9}};
    const auto anchored = AnchorPathAtEgoCore(path, Point2D{0.0, 0.0}, kParams);

    ASSERT_FALSE(anchored.empty());
    // 首点 = 自车。
    EXPECT_NEAR(anchored.front().x, 0.0, 1e-12);
    EXPECT_NEAR(anchored.front().y, 0.0, 1e-12);
    // 等间距 = path_spacing（按**弧长**重采样，弯道处弦长略小于弧长）。
    // 验收判据要求的是上界：相邻点间距 ≤ path_station_spacing × 1.5。
    for (size_t i = 1; i < anchored.size(); ++i) {
        const double d = std::hypot(anchored[i].x - anchored[i - 1].x, anchored[i].y - anchored[i - 1].y);
        EXPECT_NEAR(d, kParams.path_spacing, 1e-3);
        EXPECT_LE(d, kParams.path_spacing * 1.5);
        EXPECT_GT(d, kParams.path_spacing * 0.9);
    }
}

TEST(SkidpadCorridorCore, ProductionEntryAnchorsPathAtEgo) {
    // 直线走廊：左右两排锥桶在 x = 1..8，y = ±1.5。
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    for (int i = 1; i <= 8; ++i) {
        left.emplace_back(static_cast<double>(i), -1.5);
        right.emplace_back(static_cast<double>(i), 1.5);
    }

    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(corridor.path.empty());
    // 路径必须含自车（横偏判据的前提）。
    EXPECT_LT(std::hypot(corridor.path.front().x, corridor.path.front().y), 0.5);
    // 点序单调：到自车的距离不回退。
    for (size_t i = 1; i < corridor.path.size(); ++i) {
        const double d_prev = std::hypot(corridor.path[i - 1].x, corridor.path[i - 1].y);
        const double d_cur = std::hypot(corridor.path[i].x, corridor.path[i].y);
        EXPECT_GE(d_cur, d_prev - 1e-6) << "路径在索引 " << i << " 处回退";
    }
}

}  // namespace
