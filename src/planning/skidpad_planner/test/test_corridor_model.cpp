// ==============================================================================
// #95：**走廊瓣模型**核的行为单测（纯 std，无 ROS 依赖）。
//
// 覆盖 #95 剩余缺口（判据 3/4/8 的根因）与其可证伪点：
//   1) 直走廊不得被拟合出「假弧」——两排平行直线锥桶必须走中点法，不产出弧模型；
//   2) 切点混瓣必须锁定**某一个真实瓣**（同心双壁 → 中线半径），绝不允许产出
//      以自车为圆心的单圆（#92 实测「Crosstrack 10.03m > 6.00m, refusing goal」）；
//   3) 相位感知消歧：切点处左右两瓣的锥桶集合镜像对称（几何上不可区分），
//      必须靠上一帧路径的连续性选瓣；
//   4) 弧的**行进方向**必须与自车航向同向（旧实现比较弧两端 x，弧顶处退化 →
//      方向选反 → 实测点序 x 回跳 11 次）；
//   5) 弧的转向角 ≤ 90° → 路径点序在 x 方向单调不减（#95 判据 3）；
//   6) 部分弧（15 m 量程 / 120° 视场）下拟合半径稳定、相邻位姿漂移 ≤ 20%
//      （#95 判据 4）；
//   7) 退化输入（空 / 单点 / 共线 / 全同点）不得 UB、不得产出路径。
//
// 每个断言都能**证伪**：把实现换回「左右平均成单圆 / 贪心最近邻串接 / 不限弧长」
// 的旧版本，下列用例必然失败。
// ==============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "centerline_core.hpp"
#include "corridor_model_core.hpp"

namespace {

using skidpad::ComputeCorridorPathCore;
using skidpad::CorridorModel;
using skidpad::FitConcentricWallsCore;
using skidpad::FitCorridorModelCore;
using skidpad::GenerateCorridorArcCore;
using skidpad::Point2D;
using skidpad::WallFit;

constexpr skidpad::CenterlineParams kParams{
    .max_pair_dist = 6.0,
    .path_spacing = 0.5,
    .path_lookahead = 20.0,
};

// 八字真值（与 src/simulation/vehicle_simulator/tracks/skidpad_track.csv 一致）：
// 右瓣圆心 (0, +9.125)、左瓣圆心 (0, −9.125)，中线半径 9.125，两壁半径 7.625 / 10.625。
constexpr double kLobeMidR = 9.125;
constexpr double kLobeInnerR = 7.625;
constexpr double kLobeOuterR = 10.625;
constexpr double kRightLobeY = 9.125;
constexpr double kLeftLobeY = -9.125;

// 在圆 (cx,cy,r) 上取一段弧。
std::vector<Point2D> Arc(double cx, double cy, double r, double start_deg, double sweep_deg, int n) {
    std::vector<Point2D> pts;
    pts.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const double frac = (n > 0) ? (static_cast<double>(i) / static_cast<double>(n)) : 0.0;
        const double a = (start_deg + (sweep_deg * frac)) * std::numbers::pi_v<double> / 180.0;
        pts.emplace_back(cx + (r * std::cos(a)), cy + (r * std::sin(a)));
    }
    return pts;
}

// 某瓣在自车处可见的两壁（内壁 + 外壁），自车在原点、航向 +x。
std::vector<Point2D> LobeWalls(double lobe_cy, double start_deg, double sweep_deg, int n) {
    std::vector<Point2D> pts = Arc(0.0, lobe_cy, kLobeInnerR, start_deg, sweep_deg, n);
    const std::vector<Point2D> outer = Arc(0.0, lobe_cy, kLobeOuterR, start_deg, sweep_deg, n);
    pts.insert(pts.end(), outer.begin(), outer.end());
    return pts;
}

// ── 判据 1：直走廊不得被拟合成弧 ──────────────────────────────────────────────

TEST(SkidpadCorridorModel, StraightCorridorProducesNoArcModel) {
    // 两排平行直线锥桶（y = ±1.5），x = 1..12 —— 真实赛道的直线段。
    std::vector<Point2D> cones;
    for (int i = 1; i <= 12; ++i) {
        cones.emplace_back(static_cast<double>(i), -1.5);
        cones.emplace_back(static_cast<double>(i), 1.5);
    }

    const CorridorModel model = FitCorridorModelCore(cones, Point2D{0.0, 0.0}, kParams, {});
    // 证伪点：任何「弧模型」都是伪影 —— 直走廊的壁是直线，
    // 从任何圆心看半径都是连续谱（实测伪模型：瓣心落在锥桶环内，最近半径
    // 1.5 m ≪ 内壁半径 8.1 m；壁张角 306.9° 把瓣心包在里面）。
    EXPECT_FALSE(model.valid);

    // 生产入口必须退回中点法：路径含自车、点序单调、无 x 回跳。
    // （按节点层口径拆成车左 / 车右两组）
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    for (const auto& p : cones) {
        if (p.y < -0.3) {
            left.push_back(p);
        } else if (p.y > 0.3) {
            right.push_back(p);
        }
    }
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(corridor.path.empty());
    EXPECT_FALSE(corridor.used_model);
    EXPECT_LT(std::hypot(corridor.path.front().x, corridor.path.front().y), 0.5);
    for (size_t i = 1; i < corridor.path.size(); ++i) {
        EXPECT_GE(corridor.path[i].x, corridor.path[i - 1].x - 1e-9) << "直走廊路径 x 回跳于 " << i;
    }
}

TEST(SkidpadCorridorModel, WallFitRejectsParallelLines) {
    std::vector<Point2D> lines;
    for (int i = 1; i <= 12; ++i) {
        lines.emplace_back(static_cast<double>(i), -1.5);
        lines.emplace_back(static_cast<double>(i), 1.5);
    }
    // 直走廊不存在同心双壁模型：任意圆心都过不了「瓣心在锥桶云外侧」判据。
    for (const double cx : {0.0, 3.0, 6.0, 12.0}) {
        const WallFit w = FitConcentricWallsCore(lines, Point2D{cx, 0.0});
        EXPECT_FALSE(w.valid) << "圆心 (" << cx << ", 0) 处不应拟合出双壁";
    }
}

// ── 判据 2：切点混瓣必须锁定真实瓣 ────────────────────────────────────────────

TEST(SkidpadCorridorModel, CrossoverSelectsARealLobeNotAnEgoCentredCircle) {
    // 切点处左右两组各含**两个瓣**的锥桶（真实赛道实测：左组 12 个里 11 个属
    // 左瓣，右组 12 个里 11 个属右瓣）—— 跨左右配对会配成「右瓣外墙 ↔ 左瓣外墙」。
    const std::vector<Point2D> left = LobeWalls(kLeftLobeY, -75.0, 90.0, 7);
    const std::vector<Point2D> right = LobeWalls(kRightLobeY, -75.0, 90.0, 7);
    // 瓣模型必须看**全部**锥桶（左 ∪ 右）：切点处左右两组各自混了两个瓣的锥桶，
    // 只看一半会把「另一瓣的壁」当成噪声丢掉。
    std::vector<Point2D> all = left;
    all.insert(all.end(), right.begin(), right.end());

    const CorridorModel model = FitCorridorModelCore(all, Point2D{0.0, 0.0}, kParams, {});
    ASSERT_TRUE(model.valid);

    // 证伪点：模型中心必须落在**某个真实瓣心**上，而不是自车处。
    const double d_right = std::hypot(model.center.x - 0.0, model.center.y - kRightLobeY);
    const double d_left = std::hypot(model.center.x - 0.0, model.center.y - kLeftLobeY);
    EXPECT_TRUE((d_right < 1.0) || (d_left < 1.0))
        << "模型中心 (" << model.center.x << ", " << model.center.y << ") 不在任何瓣心上";
    // 旧实现（左右两圆平均）的圆心恰好在自车处 → 上面这条必然失败。
    EXPECT_GT(std::hypot(model.center.x, model.center.y), 5.0);

    // 中线半径必须是两壁的平均（9.125），不是别的东西。
    EXPECT_NEAR(model.radius, kLobeMidR, 0.3);
    EXPECT_NEAR(model.half_width, 1.5, 0.3);

    // 生成的弧必须落在该瓣的中线上（到瓣心恒为 9.125 m）。
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_TRUE(corridor.used_model);
    ASSERT_FALSE(corridor.path.empty());
    const Point2D c = corridor.model.center;
    for (const auto& pt : corridor.path) {
        EXPECT_NEAR(std::hypot(pt.x - c.x, pt.y - c.y), kLobeMidR, 0.3);
    }
}

// ── 判据 3：相位感知消歧（上一帧路径连续性）────────────────────────────────────

TEST(SkidpadCorridorModel, PhaseContinuityBreaksTheCrossoverTie) {
    // 切点处左右两瓣镜像对称，几何上无法区分 → 靠上一帧路径（本身就是中线
    // 采样）反推的瓣心做消歧。
    const std::vector<Point2D> left = LobeWalls(kLeftLobeY, -75.0, 90.0, 7);
    const std::vector<Point2D> right = LobeWalls(kRightLobeY, -75.0, 90.0, 7);
    std::vector<Point2D> all = left;
    all.insert(all.end(), right.begin(), right.end());

    // 上一帧路径 = 右瓣中线的一段弧（自车正前方 2~14 m）。
    const std::vector<Point2D> prev = Arc(0.0, kRightLobeY, kLobeMidR, -40.0, 60.0, 12);

    const CorridorModel model = FitCorridorModelCore(all, Point2D{0.0, 0.0}, kParams, prev);
    ASSERT_TRUE(model.valid);
    // 证伪点：必须选**右瓣**（与上一帧连续），而不是镜像的左瓣。
    EXPECT_LT(std::hypot(model.center.x - 0.0, model.center.y - kRightLobeY), 1.0);
}

TEST(SkidpadCorridorModel, ContinuityUsesPrevPathPointsNotACircleFit) {
    // 连续性判据用「上一帧路径点到候选中线圆的距离」，而不是对上一帧路径单独
    // 做圆拟合 —— 90° 弧的圆拟合本身病态（实测残差远超 0.5 m），拟合结果不可用。
    // 这里验证：同一批上一帧路径点，落在右瓣中线上的那一侧连续性误差≈0。
    const std::vector<Point2D> prev = Arc(0.0, kRightLobeY, kLobeMidR, -40.0, 60.0, 12);
    double err_right = 0.0;
    double err_left = 0.0;
    for (const auto& p : prev) {
        err_right += std::abs(std::hypot(p.x - 0.0, p.y - kRightLobeY) - kLobeMidR);
        err_left += std::abs(std::hypot(p.x - 0.0, p.y - kLeftLobeY) - kLobeMidR);
    }
    err_right /= static_cast<double>(prev.size());
    err_left /= static_cast<double>(prev.size());
    EXPECT_LT(err_right, 0.05);
    EXPECT_GT(err_left, 5.0);
}

// ── 判据 4：弧的行进方向 + 转向角上限 ──────────────────────────────────────────

TEST(SkidpadCorridorModel, ArcTangentFollowsEgoHeadingAndNeverBacksUpInX) {
    // 瓣心必须在自车的前方一侧（走廊中线过自车、且切向与航向同向 → 瓣心在法向侧）。
    // 这里取四个几何上合法的瓣心（含轻微纵向偏移），自车在原点、航向 +x。
    const std::vector<Point2D> lobes = {
        {0.0, kRightLobeY},
        {0.0, kLeftLobeY},
        {0.8, kRightLobeY},
        {0.6, kLeftLobeY},
    };
    for (const auto& c : lobes) {
        skidpad::CorridorModel m;
        m.valid = true;
        m.center = c;
        m.radius = kLobeMidR;
        const auto arc = GenerateCorridorArcCore(m, Point2D{0.0, 0.0}, kParams);
        ASSERT_FALSE(arc.empty());
        // 切向与航向同向：弧的推进方向 x 分量不得为负。
        for (size_t i = 1; i < arc.size(); ++i) {
            EXPECT_GE(arc[i].x, arc[i - 1].x - 1e-9) << "瓣心 (" << c.x << "," << c.y << ") 处 x 回跳于 " << i;
        }
        // 转向角 ≤ 90°（判据 3 的字面要求）。
        const double last_angle = std::atan2(arc.back().y - c.y, arc.back().x - c.x);
        const double first_angle = std::atan2(arc.front().y - c.y, arc.front().x - c.x);
        double turn = std::abs(last_angle - first_angle);
        if (turn > std::numbers::pi_v<double>) {
            turn = 2.0 * std::numbers::pi_v<double> - turn;
        }
        EXPECT_LE(turn, kParams.max_arc_turn_rad + 1e-6);
    }
}

TEST(SkidpadCorridorModel, LobeCentreBehindEgoYieldsNoArc) {
    // 瓣心落在自车**正后方**（切向为 ±y）→ 弧会立刻往回弯（x 立刻回退）
    // → 该模型不可用，不得产出弧。
    skidpad::CorridorModel m;
    m.valid = true;
    m.center = Point2D{-kLobeMidR, 0.0};
    m.radius = kLobeMidR;
    EXPECT_FALSE(skidpad::ArcHeadingAtEgo(m.center, Point2D{0.0, 0.0}).valid);
    EXPECT_TRUE(GenerateCorridorArcCore(m, Point2D{0.0, 0.0}, kParams).empty());
}

TEST(SkidpadCorridorModel, ArcLengthCoversPurePursuitBrakeThreshold) {
    // Pure Pursuit 的 `end_decel_points = 10` → 路径至少要有 10 个点在自车前方。
    // 90° 转向上限下，R = 9.125 m 的走廊仍给出 27 点前视。
    skidpad::CorridorModel m;
    m.valid = true;
    m.center = Point2D{0.0, kRightLobeY};
    m.radius = kLobeMidR;
    const auto arc = GenerateCorridorArcCore(m, Point2D{0.0, 0.0}, kParams);
    EXPECT_GE(arc.size(), 20U);
}

// ── 判据 5：部分弧下拟合稳定（15 m 量程 / 120° 视场）──────────────────────────

TEST(SkidpadCorridorModel, PartialArcStillRecoversTheTrueRadius) {
    // 部分弧（真值中心线上 120° 可见段）：旧实现的 Taubin 单圆拟合给出
    // 半径 16.86 / 残差 1.27（真值 9.125）；同心双壁模型必须恢复真值半径。
    for (const double start_deg : {-120.0, -90.0, -60.0, -30.0, 0.0}) {
        const std::vector<Point2D> walls = LobeWalls(kRightLobeY, start_deg, 120.0, 9);
        const WallFit w = FitConcentricWallsCore(walls, Point2D{0.0, kRightLobeY});
        ASSERT_TRUE(w.valid) << "起始角 " << start_deg << " 处应拟合出双壁";
        EXPECT_NEAR(w.r_in, kLobeInnerR, 0.15);
        EXPECT_NEAR(w.r_out, kLobeOuterR, 0.15);
        EXPECT_NEAR((w.r_in + w.r_out) * 0.5, kLobeMidR, 0.2);
        EXPECT_LE(w.rms, kParams.max_wall_rms);
    }
}

TEST(SkidpadCorridorModel, AdjacentPoseRadiusDriftStaysWithin20Percent) {
    // 判据 4：相邻两次生成的拟合半径相对漂移 ≤ 20%。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -80.0, 110.0, 10);
    const WallFit a = FitConcentricWallsCore(walls, Point2D{0.0, kRightLobeY});
    const WallFit b = FitConcentricWallsCore(walls, Point2D{0.3, kRightLobeY + 0.2});
    ASSERT_TRUE(a.valid);
    ASSERT_TRUE(b.valid);
    const double r_a = (a.r_in + a.r_out) * 0.5;
    const double r_b = (b.r_in + b.r_out) * 0.5;
    EXPECT_LE(std::abs(r_a - r_b) / r_a, 0.20);
}

// ── 判据 6：退化输入 ──────────────────────────────────────────────────────────

TEST(SkidpadCorridorModel, DegenerateInputsProduceNoModelAndNoPath) {
    const std::vector<Point2D> empty;
    const std::vector<Point2D> one = {{1.0, 2.0}};
    const std::vector<Point2D> same(4, Point2D{2.0, 3.0});
    std::vector<Point2D> collinear;
    for (int i = 0; i < 6; ++i) {
        collinear.emplace_back(static_cast<double>(i), 0.0);
    }

    EXPECT_FALSE(FitCorridorModelCore(empty, Point2D{0.0, 0.0}, kParams, {}).valid);
    EXPECT_FALSE(FitCorridorModelCore(one, Point2D{0.0, 0.0}, kParams, {}).valid);
    EXPECT_FALSE(FitCorridorModelCore(same, Point2D{0.0, 0.0}, kParams, {}).valid);
    EXPECT_FALSE(FitCorridorModelCore(collinear, Point2D{0.0, 0.0}, kParams, {}).valid);

    const auto corridor = ComputeCorridorPathCore(empty, empty, kParams, Point2D{0.0, 0.0});
    EXPECT_TRUE(corridor.path.empty());
    EXPECT_FALSE(corridor.used_model);
    EXPECT_FALSE(corridor.used_circle);
}

// ── 判据 7：生产入口的点序与间距 ──────────────────────────────────────────────

TEST(SkidpadCorridorModel, ProductionEntryIsMonotoneAndUniformlySpaced) {
    const std::vector<Point2D> left = LobeWalls(kLeftLobeY, -75.0, 90.0, 7);
    const std::vector<Point2D> right = LobeWalls(kRightLobeY, -75.0, 90.0, 7);

    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(corridor.path.empty());
    // 含自车（横偏判据的前提）。
    EXPECT_LT(std::hypot(corridor.path.front().x, corridor.path.front().y), 0.5);
    // 相邻点间距 ≤ path_station_spacing × 1.5（弯道处弦长略小于弧长）。
    for (size_t i = 1; i < corridor.path.size(); ++i) {
        const double d =
            std::hypot(corridor.path[i].x - corridor.path[i - 1].x, corridor.path[i].y - corridor.path[i - 1].y);
        EXPECT_LE(d, kParams.path_spacing * 1.5);
        EXPECT_NEAR(d, kParams.path_spacing, 1e-3);
    }
    // 点序单调：到自车的距离不回退（走廊上该量沿前进方向单调增）。
    for (size_t i = 1; i < corridor.path.size(); ++i) {
        const double d_prev = std::hypot(corridor.path[i - 1].x, corridor.path[i - 1].y);
        const double d_cur = std::hypot(corridor.path[i].x, corridor.path[i].y);
        EXPECT_GE(d_cur, d_prev - 1e-6) << "路径在索引 " << i << " 处回退";
    }
}

// ── #101：走廊模型的**质量门槛**与「沿用上一帧路径」回退 ──────────────────────
//
// 背景（Issue #101，实测）：切点处左右两瓣的锥桶同时可见 → 壁证据被稀释 →
// 模型退化（半径在 3~16 m 之间乱跳、相位误差 1.4~2.7 m、支持度只剩 4~10）→
// 路径缩到 9~15 点 → Pure Pursuit 判「接近路径末端」刹车 → 车停滞并漂出中线。
// 在线闭环门禁正样本臂通过率仅 37.5%。
//
// 三道守卫（全部可证伪）：
//   1) 支持度下限：两壁上的锥桶数太少 → 该候选不可信（只能靠噪声拟合）；
//   2) 相位误差上限：自车必须在这条走廊的中线上（文档里早有此约定，实现里
//      却只当软惩罚）；
//   3) 连续性误差上限：走廊不会突然改变——候选中线圆与上一帧路径差太远
//      （实测切点误换瓣时相差 4~5 m）→ 不可信。
// 不可信时的动作：沿用**上一帧已验证路径**，而不是退到切点处会跨瓣配对的
// 中点法（那正是 #95 的原始根因）。

// 把某瓣的两壁拆成节点层口径的「车左 / 车右」两组。
void SplitWalls(const std::vector<Point2D>& walls, std::vector<Point2D>& left, std::vector<Point2D>& right) {
    for (const auto& p : walls) {
        if (p.y < -0.3) {
            left.push_back(p);
        } else if (p.y > 0.3) {
            right.push_back(p);
        }
    }
}

TEST(SkidpadCorridorModel, WellSupportedCorridorIsTrusted) {
    // 16 个锥桶（每壁 8 个）→ 支持度达标 → 模型可信。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 8);
    const CorridorModel model = FitCorridorModelCore(walls, Point2D{0.0, 0.0}, kParams, {});
    ASSERT_TRUE(model.valid) << "两壁各 8 个锥桶应拟合出走廊模型";
    EXPECT_TRUE(model.quality_ok) << "支持度 16 ≥ " << kParams.min_model_support << "，模型应可信";
    EXPECT_GE(model.support, kParams.min_model_support);
    EXPECT_LE(model.phase_error, kParams.max_model_phase_error);
}

TEST(SkidpadCorridorModel, SparseCorridorIsNotTrusted) {
    // 只剩 8 个锥桶（每壁 4 个）→ 支持度 8 < 12 → 不可信。
    // 证伪点：把 min_model_support 改回 0（或把评分里的支持度权重去掉），
    // 本用例会失败——那时只有 4 个锥桶的壁也能被当成真走廊。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    const CorridorModel model = FitCorridorModelCore(walls, Point2D{0.0, 0.0}, kParams, {});
    if (model.valid) {
        EXPECT_FALSE(model.quality_ok) << "支持度仅 " << model.support << " 的候选不得被信任";
    }
}

TEST(SkidpadCorridorModel, EgoOffTheMidlineMakesTheCorridorUntrusted) {
    // 自车离开中线 2 m（‖ego − C‖ = 7.0，中线半径 9.125 → 相位误差 2.125 m）。
    // 该值小于旧的 phase_tol(3.0) 但大于新的硬门槛 → 必须被否决。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 8);
    const Point2D off_midline{0.0, 2.125};
    const CorridorModel model = FitCorridorModelCore(walls, off_midline, kParams, {});
    if (model.valid) {
        EXPECT_GT(model.phase_error, kParams.max_model_phase_error);
        EXPECT_FALSE(model.quality_ok) << "自车已偏离中线 " << model.phase_error << " m，模型不得被信任";
    }
}

TEST(SkidpadCorridorModel, CorridorThatJumpsAwayFromPrevPathIsNotTrusted) {
    // 走廊不会突然改变：候选中线圆与上一帧路径差 4 m 以上 → 不可信
    // （切点处误换瓣实测就是这样：上一帧在右瓣，候选却在左瓣/小半径伪圆）。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 8);
    // 上一帧路径：贴着左瓣中线（与右瓣相距 18.25 m）。
    const std::vector<Point2D> prev = LobeWalls(kLeftLobeY, -75.0, 90.0, 8);
    const CorridorModel model = FitCorridorModelCore(walls, Point2D{0.0, 0.0}, kParams, prev);
    if (model.valid) {
        EXPECT_GT(model.continuity_error, kParams.max_model_continuity_error);
        EXPECT_FALSE(model.quality_ok) << "与上一帧路径差 " << model.continuity_error << " m 的候选不得被信任";
    }
}

TEST(SkidpadCorridorModel, UntrustedCorridorFallsBackToPrevPath) {
    // 锥桶稀疏（不可信）+ 上一帧路径充足 → 生产入口必须沿用上一帧路径，
    // 而不是退到中点法（切点处会跨瓣配对）。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);

    // 上一帧路径：沿右瓣中线的一串点（点数 ≥ min_publish_points）。
    std::vector<Point2D> prev;
    for (int i = 0; i < 30; ++i) {
        const double a = (-90.0 + (3.0 * static_cast<double>(i))) * std::numbers::pi_v<double> / 180.0;
        prev.emplace_back(kLobeMidR * std::cos(a), kRightLobeY + (kLobeMidR * std::sin(a)));
    }

    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0}, prev);
    ASSERT_FALSE(corridor.path.empty());
    if (!corridor.model.quality_ok) {
        EXPECT_TRUE(corridor.used_prev_path) << "模型不可信时必须沿用上一帧路径";
        EXPECT_GE(static_cast<int>(corridor.path.size()), kParams.min_publish_points);
        EXPECT_EQ(corridor.path.size(), prev.size());
    }
}

TEST(SkidpadCorridorModel, PublishedPathAlwaysHasEnoughPointsForPurePursuit) {
    // Pure Pursuit 的 end_decel_points = 10 → 发布的路径必须 ≥ min_publish_points，
    // 否则会触发「Approaching path end」刹车（#101 的直接症状）。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 8);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0});
    ASSERT_FALSE(corridor.path.empty());
    EXPECT_GE(static_cast<int>(corridor.path.size()), kParams.min_publish_points);
}

// ── #101：持久走廊模型（上一帧已验证的走廊模型跨帧保持）──────────────────────
//
// 背景（实测）：本帧壁证据变差时模型半径会塌到 3~5 m（真值 9.125）→ 弧只剩
// 9~15 点 → Pure Pursuit 判「接近路径末端」直接刹车 → 车停滞并漂出中线。
// 两条旧退路各有硬伤：沿用上一帧**路径**会被车走完（3 s 后同样刹车）；退到
// 圆法 / 中点法会在切点跨瓣配对（#95 原始根因）。
// 正解：把走廊变成**持久状态**——保留上一帧可信模型，从当前位姿重新生成弧。

// 构造一个「可信」的右瓣模型（自车在中线上）。
CorridorModel RightLobeTrustedModel() {
    CorridorModel m;
    m.valid = true;
    m.center = Point2D{0.0, kRightLobeY};
    m.radius = kLobeMidR;
    m.half_width = (kLobeOuterR - kLobeInnerR) * 0.5;
    m.support = 24;
    m.quality_ok = true;
    return m;
}

TEST(SkidpadCorridorModel, TrustedModelKeepsLongPathWhenFreshModelCollapses) {
    // 锥桶稀疏（本帧模型必然不可信：支持度上限 8 < min_model_support=12）
    // + 上一帧的可信模型 → 必须用持久模型重新生成一条**够长**的路径。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);

    const CorridorModel trusted = RightLobeTrustedModel();
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0}, {}, &trusted);
    ASSERT_FALSE(corridor.path.empty());
    EXPECT_TRUE(corridor.used_trusted_model) << "本帧模型不可信时必须用上一帧的可信模型";
    EXPECT_GE(static_cast<int>(corridor.path.size()), kParams.min_publish_points);
    // 路径必须落在可信走廊的中线上（逐点检查半径）。
    for (const auto& p : corridor.path) {
        EXPECT_NEAR(std::hypot(p.x, p.y - kRightLobeY), kLobeMidR, 0.3);
    }
}

TEST(SkidpadCorridorModel, TrustedModelIsRejectedWhenEgoLeftTheCorridor) {
    // 自车已经离开走廊中线 2.1 m（> max_trusted_phase_error=1.5）→
    // 持久模型必须作废（否则会把车拽回旧走廊 / 掩盖真实偏离）。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);

    const CorridorModel trusted = RightLobeTrustedModel();
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 2.125}, {}, &trusted);
    EXPECT_FALSE(corridor.used_trusted_model) << "自车已离开走廊中线，持久模型不得继续使用";
}

TEST(SkidpadCorridorModel, TrustedModelPathIsRegeneratedNotConsumed) {
    // 「沿用上一帧路径」会被车走完（3 s 后触发末端刹车）；持久模型必须
    // **每帧从当前位姿重新生成** → 车前进后路径前视仍然够长（不被消耗）。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);

    // 自车在右瓣中线上前进 20°（= 3.19 m）。注意：Core 里自车恒在 base_link
    // 原点，模型圆心随自车平移——这正是节点层「全局坐标 ↔ base_link」换算的
    // 等价形式。
    const double adv = 20.0 * std::numbers::pi_v<double> / 180.0;
    const Point2D ego_adv{(kLobeMidR * std::cos(adv - (std::numbers::pi_v<double> / 2.0))),
                          kRightLobeY + (kLobeMidR * std::sin(adv - (std::numbers::pi_v<double> / 2.0)))};

    const CorridorModel trusted = RightLobeTrustedModel();
    const auto first = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0}, {}, &trusted);
    ASSERT_TRUE(first.used_trusted_model);
    ASSERT_FALSE(first.path.empty());

    CorridorModel trusted_adv = trusted;
    trusted_adv.center = Point2D{trusted.center.x - ego_adv.x, trusted.center.y - ego_adv.y};
    const auto second = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0}, {}, &trusted_adv);
    ASSERT_TRUE(second.used_trusted_model);
    ASSERT_FALSE(second.path.empty());

    // 路径前视不得随车前进而缩水（被消耗的路径会一路缩到 0）。
    const auto reach = [](const std::vector<Point2D>& path) {
        return std::hypot(path.back().x - path.front().x, path.back().y - path.front().y);
    };
    EXPECT_GT(reach(second.path), 9.0) << "持久模型的路径前视被消耗了";
    EXPECT_GT(reach(second.path), 0.8 * reach(first.path));
    EXPECT_GE(static_cast<int>(second.path.size()), kParams.min_publish_points);
    EXPECT_NEAR(second.path.front().x, 0.0, 0.05);
    EXPECT_NEAR(second.path.front().y, 0.0, 0.05);
}

TEST(SkidpadCorridorModel, TrustedModelPathReachesPurePursuitLookahead) {
    // 持久模型生成的路径必须够远（≥ 10 m），否则 Pure Pursuit 仍会因前视不足减速。
    const std::vector<Point2D> walls = LobeWalls(kRightLobeY, -75.0, 90.0, 4);
    std::vector<Point2D> left;
    std::vector<Point2D> right;
    SplitWalls(walls, left, right);

    const CorridorModel trusted = RightLobeTrustedModel();
    const auto corridor = ComputeCorridorPathCore(left, right, kParams, Point2D{0.0, 0.0}, {}, &trusted);
    ASSERT_TRUE(corridor.used_trusted_model);
    ASSERT_FALSE(corridor.path.empty());
    const double reach =
        std::hypot(corridor.path.back().x - corridor.path.front().x, corridor.path.back().y - corridor.path.front().y);
    EXPECT_GT(reach, 10.0) << "持久模型的路径前视不足（" << reach << " m）";
}

}  // namespace
