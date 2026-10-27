#pragma once

// ==============================================================================
// #95：走廊「瓣模型」核（纯 std，无 ROS 依赖，可脱 ROS 进 sanitizer 门）。
//
// 八字的一条走廊 = 两条**同心**壁（r_in / r_out）+ 夹在两者之间的中线圆。
// 旧实现把「车左 / 车右」两组锥桶各拟合一个圆再平均，隐含假设「左 = 内壁、
// 右 = 外壁」。在八字切点附近，左右两组**各自同时含有两个瓣的锥桶**（探针实测：
// 切点处左组 12 个锥桶里 11 个属左瓣，右组 12 个里 11 个属右瓣），于是：
//   - 部分弧 + 混瓣 → 代数拟合半径在 6~260 m 间漂移；
//   - 两圆平均 → 圆心 cy 被抵消到 0，生成以自车为圆心的虚假同心圆；
//   - 跨左右配对 → 配成「右瓣外墙 ↔ 左瓣外墙」，中点半径 11.85（真中线 9.125）。
//
// 本核的做法：
//   1) 先定「瓣」：一个圆心 C + 两条同心壁 → 中线半径 R_mid = (r_in + r_out)/2；
//   2) 用「自车必在走廊中线上」这一**相位事实**做接受判据
//      |‖ego − C‖ − R_mid| ≤ phase_tol，并作为切点处的消歧依据（相位感知）；
//   3) 弧线的行进方向由「切向必须与自车航向（base_link +x）同向」决定，
//      而不是靠比较弧线两端的 x（弧顶处该比较退化，会把方向选反 → 点序回跳）。
//
// 判据说明（每条都能证伪）：
//   - 两壁必须张成一个**弧**（弧角 ≥ min_wall_arc_rad）→ 排除直走廊被拟合出的
//     「径向切片伪壁」（直走廊的两排锥桶在任何圆心下都张不出弧）；
//   - 壁半径残差 RMS ≤ max_wall_rms → 壁必须真的是圆弧；
//   - 相位误差 ≤ phase_tol → 自车必须在这条走廊的中线上。
// ==============================================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <span>
#include <vector>

#include "centerline_core.hpp"

namespace skidpad {

// #95：一条走廊的模型（瓣圆心 + 中线半径 + 壁几何）。
struct CorridorModel {
    bool valid{false};
    Point2D center;            // 瓣圆心（base_link）
    double radius{0.0};        // 中线半径 R_mid = (r_in + r_out) / 2
    double half_width{0.0};    // 壁到中线的距离
    int support{0};            // 落在两壁上的锥桶数
    double wall_rms{0.0};      // 壁半径残差 RMS
    double arc_span_rad{0.0};  // 两壁在 C 处张成的弧角
    double phase_error{0.0};   // |‖ego − C‖ − R_mid|
    int gap_count{0};          // 两壁之间的锥桶数（真走廊应为 0）
    // #101：候选与**上一帧路径**的连续性误差（上一帧路径各点到本候选中线圆
    // 的平均距离差，米）。走廊不会突然改变：该值过大说明这条候选描述的不是
    // 自车所在的那条走廊（切点处混瓣时会拟合出半径 3~5 m 的伪圆）。
    double continuity_error{0.0};
    // #101：候选是否通过**质量门槛**（相位误差 ≤ max_model_phase_error、
    // 支持度 ≥ min_model_support、连续性误差 ≤ max_model_continuity_error）。
    // false = 「有走廊候选但都不可信」——此时应沿用上一帧已验证路径，而不是
    // 退到会在切点跨瓣配对的中点法。
    bool quality_ok{true};
};

// #95：给定瓣圆心时，两条同心壁的拟合结果。
struct WallFit {
    bool valid{false};
    double r_in{0.0};
    double r_out{0.0};
    int support{0};
    int gap_count{0};  // 落在两壁**之间**的锥桶数（走廊内部的锥桶）
    double rms{0.0};
    double arc_span_rad{0.0};
};

inline constexpr int kWallHistBins = 64;
inline constexpr double kWallHistBin = 0.5;        // 半径直方图 bin（米）
inline constexpr double kWallRefineTol = 0.5;      // 壁半径归并窗口（米）
inline constexpr double kWallMinSeparation = 2.0;  // 两壁半径最小间距（米）
inline constexpr double kHalfPi = 1.5707963267948966;

// #95：弧在自车处的**切向**与「保持 x 单调不减」的可行转向角上限。
//
// 走廊中线过自车、自车沿中线行驶 → 弧在自车处的切向必须与航向（base_link +x）
// 同向；瓣心因此位于自车的法向一侧。弧的切向从 α0 出发、按 dir 方向旋转 turn，
// 要让路径点的 x 单调不减，切向必须始终落在 [−90°, +90°] 内：
//   dir = +1（θ 增大）→ turn ≤ 90° − α0
//   dir = −1（θ 减小）→ turn ≤ α0 + 90°
// 上限 ≤ 0 表示这个瓣心落在自车正后方（切向为 ±y，车会立刻往回打）→ 模型不可用。
struct ArcHeading {
    bool valid{false};
    double dir{1.0};       // +1 = θ 增大，−1 = θ 减小
    double alpha0{0.0};    // 自车处切向与 +x 的夹角（rad）
    double max_turn{0.0};  // 保持 x 单调不减的转向角上限（rad）
};

inline ArcHeading ArcHeadingAtEgo(Point2D center, Point2D ego) {
    ArcHeading h;
    const double theta0 = std::atan2(ego.y - center.y, ego.x - center.x);
    h.dir = (std::sin(theta0) < 0.0) ? 1.0 : -1.0;
    h.alpha0 = (h.dir > 0.0) ? std::atan2(std::cos(theta0), -std::sin(theta0))
                             : std::atan2(-std::cos(theta0), std::sin(theta0));
    h.max_turn = (h.dir > 0.0) ? (kHalfPi - h.alpha0) : (h.alpha0 + kHalfPi);
    h.valid = h.max_turn > 0.0;
    return h;
}

// #95：给定圆心 C，估计「以 C 为心、同心的两条壁」。
//
// 半径直方图上找两个峰（壁半径），再在峰附近 ±kWallRefineTol 内取均值精修；
// 壁半径残差与弧角跨度同时作为「这条壁是不是真的圆弧」的判据。
inline WallFit FitConcentricWallsCore(std::span<const Point2D> cones, Point2D center) {
    WallFit fit;
    if (cones.size() < 4) {
        return fit;
    }

    std::array<int, kWallHistBins> hist{};
    std::vector<double> radii;
    radii.reserve(cones.size());
    for (const auto& p : cones) {
        const double r = std::hypot(p.x - center.x, p.y - center.y);
        if (!std::isfinite(r) || r >= kWallHistBins * kWallHistBin) {
            continue;
        }
        radii.push_back(r);
        ++hist[static_cast<std::size_t>(r / kWallHistBin)];
    }
    if (radii.size() < 4) {
        return fit;
    }

    int peak_a = 0;
    for (int b = 1; b < kWallHistBins; ++b) {
        if (hist[static_cast<std::size_t>(b)] > hist[static_cast<std::size_t>(peak_a)]) {
            peak_a = b;
        }
    }
    if (hist[static_cast<std::size_t>(peak_a)] < 2) {
        return fit;
    }

    int peak_b = -1;
    for (int b = 0; b < kWallHistBins; ++b) {
        if (static_cast<double>(std::abs(b - peak_a)) * kWallHistBin < kWallMinSeparation) {
            continue;
        }
        if (hist[static_cast<std::size_t>(b)] < 2) {
            continue;
        }
        if (peak_b < 0 || hist[static_cast<std::size_t>(b)] > hist[static_cast<std::size_t>(peak_b)]) {
            peak_b = b;
        }
    }
    if (peak_b < 0) {
        return fit;
    }

    const int lo = std::min(peak_a, peak_b);
    const int hi = std::max(peak_a, peak_b);
    const double seed_in = static_cast<double>(lo) * kWallHistBin;
    const double seed_out = static_cast<double>(hi) * kWallHistBin;

    double sum_in = 0.0;
    double sum_out = 0.0;
    int cnt_in = 0;
    int cnt_out = 0;
    for (const double r : radii) {
        if (std::abs(r - seed_in) <= kWallRefineTol) {
            sum_in += r;
            ++cnt_in;
        } else if (std::abs(r - seed_out) <= kWallRefineTol) {
            sum_out += r;
            ++cnt_out;
        }
    }
    if (cnt_in < 2 || cnt_out < 2) {
        return fit;
    }
    const double r_in = sum_in / static_cast<double>(cnt_in);
    const double r_out = sum_out / static_cast<double>(cnt_out);
    if (r_out - r_in < kWallMinSeparation) {
        return fit;
    }

    double sse = 0.0;
    int support = 0;
    int gap_count = 0;
    std::vector<double> angles;
    angles.reserve(radii.size());
    for (const auto& p : cones) {
        const double r = std::hypot(p.x - center.x, p.y - center.y);
        if (!std::isfinite(r)) {
            continue;
        }
        double err = 0.0;
        bool on_wall = false;
        if (std::abs(r - r_in) <= kWallRefineTol) {
            err = r - r_in;
            on_wall = true;
        } else if (std::abs(r - r_out) <= kWallRefineTol) {
            err = r - r_out;
            on_wall = true;
        } else if (r > r_in + kWallRefineTol && r < r_out - kWallRefineTol) {
            // 走廊**内部**的锥桶：真走廊的两壁之间应当是空的。
            // 直走廊（两排平行直线）从任何圆心看半径都是连续谱 → 这里必然非空，
            // 于是「直走廊被拟合出一条假弧」这条伪影被挡掉。
            ++gap_count;
        }
        if (!on_wall) {
            continue;
        }
        sse += err * err;
        ++support;
        angles.push_back(std::atan2(p.y - center.y, p.x - center.x));
    }
    if (support < 4 || angles.size() < 4) {
        return fit;
    }
    if (gap_count > std::max(1, support / 5)) {
        return fit;
    }
    // 瓣心必须在锥桶云**外侧**：走廊内壁与瓣心之间不该再有锥桶。
    // 直走廊被拟合成「以自车前方某点为心的小圆」时，最近锥桶会落在内壁半径
    // 以内（实测 1.5 m ≪ 8.1 m）→ 这条判据把该伪影挡掉。
    double min_radius = std::numeric_limits<double>::max();
    for (const double r : radii) {
        min_radius = std::min(min_radius, r);
    }
    if (min_radius < r_in - kWallRefineTol) {
        return fit;
    }

    // 弧角跨度 = 2π − 最大间隙（处理 ±π 回绕）
    std::ranges::sort(angles);
    double max_gap = 0.0;
    for (size_t i = 0; i < angles.size(); ++i) {
        const double a0 = angles[i];
        const double a1 = angles[(i + 1) % angles.size()];
        // 末尾一段跨过 ±π：间隙 = 首角 + 2π − 末角（不是反向）。
        const double gap = (i + 1 == angles.size())
                               ? ((angles.front() + (2.0 * std::numbers::pi_v<double>)) - angles.back())
                               : (a1 - a0);
        max_gap = std::max(max_gap, gap);
    }

    fit.valid = true;
    fit.r_in = r_in;
    fit.r_out = r_out;
    fit.support = support;
    fit.gap_count = gap_count;
    fit.rms = std::sqrt(sse / static_cast<double>(support));
    fit.arc_span_rad = (2.0 * std::numbers::pi_v<double>)-max_gap;
    return fit;
}

// #95：走廊模型搜索（相位感知）。
//
// 候选瓣圆心按「距自车的距离 R × 方位角 φ」参数化：自车在中线上 → 圆心必在
// 自车的法向附近，φ 只需覆盖 [20°, 340°]。先用粗网格找支持度最高的几个解，
// 再在它们附近细化，最后按「支持度 → 相位误差 → 与上一帧模型的连续性」择优。
//
// 切点处左右两瓣的锥桶集合是**镜像对称**的（几何上无法区分），因此必须有
// 相位/连续性依据：上一帧路径本身就是中线采样，其圆拟合给出上一拍的瓣圆心，
// #101：走廊模型的**质量体检**（对最终选中的候选做否决式判定）。
//
// 三条判据（全部可证伪；阈值见 CenterlineParams）：
//   1) 支持度：两壁上的锥桶数 ≥ min_model_support——锥桶太少时「双壁」只是噪声；
//   2) 相位误差：|‖ego − C‖ − R_mid| ≤ max_model_phase_error——自车必须在这条
//      走廊的中线上（文件头注释早已把它写成接受判据，实现里此前只当软惩罚）；
//   3) 连续性误差：候选中线圆与**上一帧路径**的平均距离差 ≤
//      max_model_continuity_error——走廊不会突然改变（切点误换瓣实测差 4~5 m）。
inline bool PassesQualityGate(const CorridorModel& m, const CenterlineParams& p) {
    if (!m.valid) {
        return false;
    }
    if (m.support < p.min_model_support) {
        return false;
    }
    if (m.phase_error > p.max_model_phase_error) {
        return false;
    }
    if (m.continuity_error > p.max_model_continuity_error) {
        return false;
    }
    return true;
}

// 用它做同支持度候选之间的消歧。
inline CorridorModel FitCorridorModelCore(std::span<const Point2D> cones, Point2D ego, const CenterlineParams& p,
                                          std::span<const Point2D> prev_path) {
    CorridorModel best;
    if (cones.size() < 4) {
        return best;
    }

    // 相位/连续性先验：上一帧路径本身就是中线采样 → 它到候选中线圆的距离就是
    // 「连续性误差」。**不**对上一帧路径单独做圆拟合（90° 弧的圆拟合本身病态，
    // 实测残差远超 0.5 m，拟合结果不可用）。
    const bool has_prev = (prev_path.size() >= 4);
    auto continuity_error = [&](const CorridorModel& m) {
        if (!has_prev) {
            return 0.0;
        }
        double sum = 0.0;
        for (const auto& p : prev_path) {
            sum += std::abs(std::hypot(p.x - m.center.x, p.y - m.center.y) - m.radius);
        }
        return sum / static_cast<double>(prev_path.size());
    };

    std::vector<CorridorModel> pool;
    pool.reserve(2048);

    auto eval = [&](double radius, double phi, std::vector<CorridorModel>& sink) {
        if (!(radius > 0.0) || !std::isfinite(radius) || !std::isfinite(phi)) {
            return;
        }
        const Point2D c{ego.x + (radius * std::cos(phi)), ego.y + (radius * std::sin(phi))};
        const WallFit w = FitConcentricWallsCore(cones, c);
        if (!w.valid) {
            return;
        }
        const double half = (w.r_out - w.r_in) * 0.5;
        if (half < 0.5 || half > p.max_half_width) {
            return;
        }
        if (w.rms > p.max_wall_rms) {
            return;
        }
        if (w.arc_span_rad < p.min_wall_arc_rad || w.arc_span_rad > p.max_wall_arc_rad) {
            return;
        }
        CorridorModel m;
        m.center = c;
        m.radius = (w.r_in + w.r_out) * 0.5;
        m.half_width = half;
        m.support = w.support;
        m.wall_rms = w.rms;
        m.arc_span_rad = w.arc_span_rad;
        m.gap_count = w.gap_count;
        m.phase_error = std::abs(std::hypot(ego.x - c.x, ego.y - c.y) - m.radius);
        if (m.radius < p.min_model_radius || m.radius > p.max_model_radius) {
            return;
        }
        if (m.phase_error > p.phase_tol) {
            return;
        }
        // 走廊的长宽比：半径 / 半宽。比值过大说明这条「走廊」其实是直线
        // （直走廊被拟合成的大半径伪弧实测 15.4，真走廊 5.6~7.6）→ 不得采用。
        if (m.half_width > 0.0 && (m.radius / m.half_width) > p.max_radius_width_ratio) {
            return;
        }
        // 瓣心必须在自车的前方一侧（切向与航向同向）——瓣心落在自车正后方时
        // 弧会立刻往回弯（x 立刻回退），该模型不可用。
        if (!ArcHeadingAtEgo(c, ego).valid) {
            return;
        }
        m.valid = true;
        sink.push_back(m);
    };

    // 粗网格
    for (double r = p.min_model_radius; r <= p.max_model_radius; r += 1.0) {
        for (int deg = 20; deg <= 340; deg += 20) {
            eval(r, static_cast<double>(deg) * std::numbers::pi_v<double> / 180.0, pool);
        }
    }
    if (pool.empty()) {
        return best;
    }

    // 细化：取支持度最高的 3 个粗解，在其附近细扫
    std::vector<CorridorModel> seeds = pool;
    std::ranges::sort(seeds, [](const CorridorModel& a, const CorridorModel& b) { return a.support > b.support; });
    if (seeds.size() > 3) {
        seeds.resize(3);
    }
    std::vector<CorridorModel> refined;
    refined.reserve(2048);
    for (const auto& seed : seeds) {
        const double r0 = std::hypot(seed.center.x - ego.x, seed.center.y - ego.y);
        const double phi0 = std::atan2(seed.center.y - ego.y, seed.center.x - ego.x);
        for (double dr = -1.5; dr <= 1.5001; dr += 0.25) {
            for (int k = -10; k <= 10; ++k) {
                eval(r0 + dr, phi0 + (static_cast<double>(k) * 2.0 * std::numbers::pi_v<double> / 180.0), refined);
            }
        }
    }

    std::vector<const CorridorModel*> all;
    all.reserve(pool.size() + refined.size());
    for (const auto& m : pool) {
        all.push_back(&m);
    }
    for (const auto& m : refined) {
        all.push_back(&m);
    }

    // #101：质量门槛采用**否决**语义（不是候选过滤器）——先按原评分选出最佳解，
    // 再对它做质量体检；不达标就整条候选作废（调用方改走上一帧已验证路径）。
    //
    // 为什么不用「过滤器」：相位误差几乎恒等于 0（候选的圆心是以自车为中心
    // 参数化的，半径取自壁拟合），把它当过滤条件会在自车已偏离中线时**反向
    // 选出更差的伪候选**（实测：真候选因相位 2.1 m 被否 → 退到一个支持度更高
    // 但几何无关的圆）。否决语义不会制造这种降级。
    //
    // 背景：切点处两瓣锥桶混入 → 壁证据被稀释 → 支持度只剩 4~10 的伪模型也能
    // 胜出（实测半径在 3~16 m 之间乱跳、相位误差 1.4~2.7 m）→ 路径缩到 9~15
    // 点 → Pure Pursuit 刹车 → 车停滞漂移（在线闭环门禁通过率仅 37.5%）。

    // ── 择优 ────────────────────────────────────────────────────────────────
    // 评分 = 支持度 − 相位惩罚。自车必在中线上 → 相位误差应当≈0；真模型的
    // 相位误差实测 0.01~0.42 m，而「左右混瓣」的伪模型相位误差往往更大。
    // 取近似并列（差距 ≤ 0.5）的候选作为决赛集：切点处左右两瓣的锥桶集合是
    // 镜像对称的（几何上不可区分），必须靠上一帧模型的连续性消歧。
    constexpr double kPhaseWeight = 3.0;       // 1 m 相位误差折算 3 个支持点
    constexpr double kContinuityWeight = 1.0;  // 1 m 连续性误差折算 1 个支持点
    auto primary_score = [&](const CorridorModel& m) {
        return static_cast<double>(m.support) - (kPhaseWeight * m.phase_error) -
               (kContinuityWeight * continuity_error(m));
    };
    double best_score = -std::numeric_limits<double>::max();
    for (const auto* m : all) {
        best_score = std::max(best_score, primary_score(*m));
    }

    const CorridorModel* chosen = nullptr;
    double chosen_score = std::numeric_limits<double>::max();
    for (const auto* m : all) {
        if (primary_score(*m) < best_score - 0.5) {
            continue;
        }
        // 决赛集内：优先与上一帧路径连续、相位误差小的候选（切点消歧）。
        const double tie = m->phase_error + (kContinuityWeight * continuity_error(*m));
        if (tie < chosen_score) {
            chosen_score = tie;
            chosen = m;
        }
    }
    if (chosen == nullptr) {
        best.continuity_error = continuity_error(best);
        best.quality_ok = PassesQualityGate(best, p);
        return best;
    }
    best = *chosen;
    best.continuity_error = continuity_error(best);
    best.quality_ok = PassesQualityGate(best, p);
    return best;
}

// #95：由走廊模型生成**从自车起步**的弧。
//
// 行进方向由「弧在自车处的切向必须与自车航向（base_link +x）同向」决定：
//   P(θ) = C + R·(cos θ, sin θ) → dP/dθ = R·(−sin θ, cos θ)
//   θ 增大时切向 x 分量 = −R·sin θ0，要为正 → sin θ0 < 0。
// 旧实现比较弧线两端的 x（first_x(+1) vs first_x(−1)），在弧顶处该比较退化
// （切向恰为 ±x）→ 方向选反 → 实测点序 x 回跳 11 次。
//
// 弧的**转向角上限**取 max_arc_turn_rad（默认 90°）：切向从 +x 转到 +y 的过程
// 里 x 单调不减；再往后弯，x 必然回退（与 #95 判据 3「相邻点 x 不得回跳 >0.05 m」
// 冲突——弯道上 20 m 前视对应 132° 弧，x 必然回头）。按 90° 截断后，
// R=8.7 m 的走廊仍给出 13.7 m / 27 点前视，远高于 Pure Pursuit 的
// `end_decel_points = 10` 刹车阈值。
inline std::vector<Point2D> GenerateCorridorArcCore(const CorridorModel& m, Point2D ego, const CenterlineParams& p) {
    if (!m.valid || !(m.radius > 0.0) || !(p.path_spacing > 0.0) || !(p.path_lookahead > 0.0)) {
        return {};
    }
    const ArcHeading head = ArcHeadingAtEgo(m.center, ego);
    if (!head.valid) {
        return {};  // 瓣心在自车正后方 → 弧会立刻往回弯，不可用
    }
    const double turn = std::min(p.max_arc_turn_rad, head.max_turn);
    if (!(turn > 0.0)) {
        return {};
    }
    const double arc_step = p.path_spacing / m.radius;
    const double theta0 = std::atan2(ego.y - m.center.y, ego.x - m.center.x);
    const double dir = head.dir;
    const double max_arc_len = std::min(p.path_lookahead, m.radius * turn);
    const int steps = static_cast<int>(max_arc_len / p.path_spacing);
    if (steps <= 0) {
        return {};
    }

    std::vector<Point2D> path;
    path.reserve(static_cast<size_t>(steps) + 1);
    for (int i = 1; i <= steps; ++i) {
        const double a = theta0 + (dir * arc_step * static_cast<double>(i));
        path.emplace_back(m.center.x + (m.radius * std::cos(a)), m.center.y + (m.radius * std::sin(a)));
    }
    return path;
}

// #95：走廊中线路径的产出结果（供节点层与单测读取判据来源）。
struct CorridorPathResult {
    // base_link 系，含自车、点序单调、等间距
    std::vector<Point2D> path;
    // true = 走瓣模型（相位感知同心双壁）
    bool used_model{false};
    // true = 走圆法（相位感知单圆）
    bool used_circle{false};
    // #101：true = 路径**用到**了上一帧路径（整条沿用，或用它补齐前视）
    bool used_prev_path{false};
    // #101：true = 本帧走廊模型不可信，改用**上一帧已验证的走廊模型**
    // （持久走廊状态）从当前位姿重新生成了弧。
    bool used_trusted_model{false};
    // 选中的走廊模型（未选中时 valid=false）
    CorridorModel model;
};

// #101：**持久走廊模型**的可用性校验。
//
// 「上一帧已验证的走廊模型」能不能继续用，看的是**自车是否还在这条走廊的
// 中线上**：|‖ego − C‖ − R_mid| ≤ max_trusted_phase_error。
//
// 为什么用相位而不是「模型是否还拟合得上」：模型是上一帧的，壁证据本帧已经
// 变差（这正是要持久化的原因）；而自车始终在走廊中线上——只要车还在走廊里，
// 这条模型就是对的（走廊不会突然改变）。反过来，若车已经偏离中线超过容差，
// 说明要么车跑偏了、要么走廊已经切换（如八字切点），此时必须作废，不能拿
// 旧模型把车拽回旧走廊。
inline bool TrustedModelUsableCore(const CorridorModel& m, Point2D ego, const CenterlineParams& p) {
    if (!m.valid || !(m.radius > 0.0) || !std::isfinite(m.center.x) || !std::isfinite(m.center.y)) {
        return false;
    }
    const double phase = std::abs(std::hypot(ego.x - m.center.x, ego.y - m.center.y) - m.radius);
    return phase <= p.max_trusted_phase_error;
}

// #95：走廊中线的**生产入口**（节点层只调这一个）。
//
// 顺序：瓣模型法（相位感知同心双壁）→ **上一帧可信模型**（#101 持久走廊状态）
// → 中点法回退。三条路径都保证「含自车 + 点序单调 + 等间距」，避免
// Pure Pursuit 拒解 / 「Approaching path end」末端刹车。
//
// #101：trusted_model 是**上一帧已验证**的走廊模型（调用方持有，坐标为
// base_link；模型不可信时用它从当前位姿重新生成弧）。为 nullptr 时行为与
// #95 一致（不含持久化分支）。
inline CorridorPathResult ComputeCorridorPathCore(std::span<const Point2D> left, std::span<const Point2D> right,
                                                  const CenterlineParams& p, Point2D ego = Point2D{0.0, 0.0},
                                                  std::span<const Point2D> prev_path = {},
                                                  const CorridorModel* trusted_model = nullptr) {
    CorridorPathResult out;

    // ── 1) 瓣模型法 ──────────────────────────────────────────────────────────
    // 左右两组合起来才是「本帧看到的全部锥桶」——瓣模型必须看**全部**锥桶，
    // 否则切点处会漏掉另一半瓣的壁（那正是跨瓣混淆的来源）。
    std::vector<Point2D> all;
    all.reserve(left.size() + right.size());
    all.insert(all.end(), left.begin(), left.end());
    all.insert(all.end(), right.begin(), right.end());

    const CorridorModel model = FitCorridorModelCore(all, ego, p, prev_path);
    const bool prev_path_usable = !prev_path.empty() && static_cast<int>(prev_path.size()) >= p.min_publish_points;

    // ── 1a) 模型**可信**（相位误差与支持度都达标）→ 走瓣模型 ──────────────────
    std::vector<Point2D> fresh_path;
    bool fresh_extended = false;
    if (model.valid && model.quality_ok) {
        const std::vector<Point2D> arc = GenerateCorridorArcCore(model, ego, p);
        if (!arc.empty()) {
            std::vector<Point2D> anchored = AnchorPathAtEgoCore(arc, ego, p);
            if (!anchored.empty()) {
                // #101：模型弧太短时用**上一帧路径**补齐前视（与中点法同一机制）。
                // 切点处两瓣锥桶混入 → 壁证据被稀释 → 弧退化（实测 9~15 点）→
                // Pure Pursuit 判「接近路径末端」直接刹车 → 车停滞。
                fresh_path = ExtendWithPrevPathCore(anchored, prev_path, ego, p);
                fresh_extended = (fresh_path.size() != anchored.size());
            }
        }
    }
    // 点数不足 min_publish_points 的短路径会让 Pure Pursuit 判「接近路径末端」
    // 而刹车（实测退化帧 9~19 点、健康帧 19~43 点）→ 先看有没有更稳的替代。
    const bool fresh_ok = !fresh_path.empty() && static_cast<int>(fresh_path.size()) >= p.min_publish_points;
    if (fresh_ok) {
        out.used_model = true;
        out.used_prev_path = fresh_extended;
        out.model = model;
        out.path = std::move(fresh_path);
        return out;
    }

    // ── 1b) 本帧模型不可信 → 用**上一帧已验证的走廊模型**（持久走廊状态）─────
    //
    // 从**当前自车位姿**重新生成弧：弧长只由模型（瓣心 + 中线半径）决定，
    // 不受本帧壁证据好坏影响（实测退化帧半径会塌到 3~5 m → 弧只剩 9~15 点），
    // 且每帧都是新路径（不会像「沿用上一帧路径」那样被车走完后触发末端刹车）。
    if (trusted_model != nullptr && TrustedModelUsableCore(*trusted_model, ego, p)) {
        const std::vector<Point2D> arc = GenerateCorridorArcCore(*trusted_model, ego, p);
        if (!arc.empty()) {
            std::vector<Point2D> anchored = AnchorPathAtEgoCore(arc, ego, p);
            if (!anchored.empty()) {
                std::vector<Point2D> extended = ExtendWithPrevPathCore(anchored, prev_path, ego, p);
                if (!extended.empty() && static_cast<int>(extended.size()) >= p.min_publish_points) {
                    out.used_model = true;
                    out.used_trusted_model = true;
                    out.used_prev_path = (extended.size() != anchored.size());
                    out.model = *trusted_model;
                    out.path = std::move(extended);
                    return out;
                }
            }
        }
    }

    // ── 1c) 有走廊候选但**质量不达标**（切点处壁证据不足 → 伪模型）──────────
    // 沿用上一帧已验证的路径；不要退到中点法——切点处左右两组锥桶分属不同瓣，
    // 中点法会跨瓣配对（这正是 #95 的原始根因），会把车带离中线。
    if (model.valid && !model.quality_ok && prev_path_usable) {
        out.path.assign(prev_path.begin(), prev_path.end());
        out.used_prev_path = true;
        return out;
    }

    // ── 1d) 本帧模型可信但弧太短，且没有可用的持久模型 ──────────────────────
    // 仍然发布这条几何正确的短路径（而不是退到切点处会跨瓣配对的中点法）——
    // 短路径只会让 PP 减速，几何错误的路径会让车冲出中线。
    if (!fresh_path.empty()) {
        out.used_model = true;
        out.model = model;
        out.path = std::move(fresh_path);
        return out;
    }

    // ── 2) 圆法（相位感知单圆）────────────────────────────────────────────────
    std::vector<Point2D> path = ComputeCircleCenterlineCore(left, right, p, ego);
    if (!path.empty()) {
        out.used_circle = true;
        out.path = std::move(path);
        return out;
    }

    // ── 3) 中点法回退 ────────────────────────────────────────────────────────
    path = ComputeCenterlineCore(left, right, p);
    if (path.empty()) {
        return out;
    }
    // 点序单调连续 —— 先用「互为最近邻」过滤掉单向配对产生的伪中点，
    // 再做方向感知重排（不丢点：遗留输出是中点集合的一个排列），
    // 最后锚到自车并等距重采样。
    std::vector<Point2D> mutual = AppendMutualMidpointsCore(left, right, p.max_pair_dist);
    if (!mutual.empty()) {
        path = std::move(mutual);
    }
    std::vector<Point2D> ordered = OrderAlongCorridorCore(path, ego, p.max_pair_dist);
    if (ordered.empty()) {
        return out;
    }
    out.path = AnchorPathAtEgoCore(ordered, ego, p);
    // 用上一帧路径补齐前视长度（紧弯上传感器只看得到 ~5 m 走廊）。
    out.path = ExtendWithPrevPathCore(out.path, prev_path, ego, p);
    return out;
}

}  // namespace skidpad
