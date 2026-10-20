#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "line_fit_core.hpp"

namespace {

// 模拟外部结构体（验证 Position2DLike 鸭子类型支持，无需 ROS 消息头）
struct CustomBaseLinkCone {
    struct Position {
        double x{0.0};
        double y{0.0};
        double z{0.0};
    } position_base_link;
};

// 构造已知直线的黄金样本锥桶集：y = m * x + b
std::vector<slp_core::Point2D> MakeLinePoints(double m, double b, double start_x, double end_x, double step) {
    std::vector<slp_core::Point2D> pts;
    for (double x = start_x; x <= end_x + 1e-9; x += step) {
        pts.push_back(slp_core::Point2D{x, m * x + b});
    }
    return pts;
}

}  // namespace

// ── 1. 黄金样本：平行双直线拟合一致性 ──────────────────────────────────────────

TEST(LineFitCoreTest, GoldenSampleParallelLinesConsistency) {
    slp_core::LineDetectorConfig cfg;
    cfg.enable_hough = true;
    cfg.hough_rho_resolution = 0.20;
    cfg.hough_min_inlier_ratio = 0.4;
    cfg.ransac_max_abs_slope = 0.3;

    // 左直线：m = 0.05, b = -1.5；右直线：m = 0.05, b = 1.5
    const double expected_m = 0.05;
    const double expected_b_left = -1.5;
    const double expected_b_right = 1.5;

    auto left_pts = MakeLinePoints(expected_m, expected_b_left, 1.0, 15.0, 1.0);
    auto right_pts = MakeLinePoints(expected_m, expected_b_right, 1.0, 15.0, 1.0);

    // (1) 最小二乘法 (LSQ) 拟合
    slp_core::LineParams lsq_left = slp_core::LeastSquaresFitCore(std::span<const slp_core::Point2D>{left_pts}, cfg);
    slp_core::LineParams lsq_right = slp_core::LeastSquaresFitCore(std::span<const slp_core::Point2D>{right_pts}, cfg);
    EXPECT_TRUE(lsq_left.valid);
    EXPECT_TRUE(lsq_right.valid);
    EXPECT_NEAR(lsq_left.slope, expected_m, 1e-3);
    EXPECT_NEAR(lsq_left.intercept, expected_b_left, 1e-3);
    EXPECT_NEAR(lsq_right.slope, expected_m, 1e-3);
    EXPECT_NEAR(lsq_right.intercept, expected_b_right, 1e-3);

    // (2) RANSAC（固定种子确定性拟合）
    constexpr std::uint64_t kSeed = 20261019ULL;
    slp_core::LineParams ransac_left =
        slp_core::RansacFitCore(std::span<const slp_core::Point2D>{left_pts}, cfg, kSeed);
    slp_core::LineParams ransac_right =
        slp_core::RansacFitCore(std::span<const slp_core::Point2D>{right_pts}, cfg, kSeed);
    EXPECT_TRUE(ransac_left.valid);
    EXPECT_TRUE(ransac_right.valid);
    EXPECT_NEAR(ransac_left.slope, expected_m, 0.02);
    EXPECT_NEAR(ransac_left.intercept, expected_b_left, 0.1);
    EXPECT_NEAR(ransac_right.slope, expected_m, 0.02);
    EXPECT_NEAR(ransac_right.intercept, expected_b_right, 0.1);

    // (3) Hough 变换拟合
    slp_core::LineParams hough_left = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{left_pts}, cfg);
    slp_core::LineParams hough_right = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{right_pts}, cfg);
    EXPECT_TRUE(hough_left.valid);
    EXPECT_TRUE(hough_right.valid);
    EXPECT_NEAR(hough_left.slope, expected_m, 0.05);
    EXPECT_NEAR(hough_left.intercept, expected_b_left, 0.2);
    EXPECT_NEAR(hough_right.slope, expected_m, 0.05);
    EXPECT_NEAR(hough_right.intercept, expected_b_right, 0.2);

    // 校验线型质量评价器
    EXPECT_TRUE(slp_core::IsLineGoodCore(lsq_left, std::span<const slp_core::Point2D>{left_pts}, 0.3, cfg));
    EXPECT_TRUE(slp_core::IsLineGoodCore(ransac_left, std::span<const slp_core::Point2D>{left_pts}, 0.3, cfg));
    EXPECT_TRUE(slp_core::IsLineGoodCore(hough_left, std::span<const slp_core::Point2D>{left_pts}, 0.3, cfg));
}

// ── 2. 退化与病态几何分支 ───────────────────────────────────────────────────

TEST(LineFitCoreTest, DegenerateGeometryHandling) {
    slp_core::LineDetectorConfig cfg;
    std::vector<slp_core::Point2D> empty_pts;
    std::vector<slp_core::Point2D> single_pt{{2.0, 1.0}};

    // 空集
    EXPECT_FALSE(slp_core::LeastSquaresFitCore(std::span<const slp_core::Point2D>{empty_pts}, cfg).valid);
    EXPECT_FALSE(slp_core::RansacFitCore(std::span<const slp_core::Point2D>{empty_pts}, cfg).valid);
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{empty_pts}, cfg).valid);

    // 单点 (<2)
    EXPECT_FALSE(slp_core::LeastSquaresFitCore(std::span<const slp_core::Point2D>{single_pt}, cfg).valid);
    EXPECT_FALSE(slp_core::RansacFitCore(std::span<const slp_core::Point2D>{single_pt}, cfg).valid);
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{single_pt}, cfg).valid);

    // 全同 x 坐标点集（denom == 0）
    std::vector<slp_core::Point2D> same_x_pts{{3.0, 1.0}, {3.0, 2.0}, {3.0, 3.0}};
    slp_core::LineParams same_x_fit =
        slp_core::LeastSquaresFitCore(std::span<const slp_core::Point2D>{same_x_pts}, cfg);
    EXPECT_FALSE(same_x_fit.valid);  // 触发 denom 守卫，不除零崩溃

    // 负 x 间隔参数（ransac_min_x_spread < 0）
    cfg.ransac_min_x_spread = -1.0;
    auto normal_pts = MakeLinePoints(0.1, 1.0, 1.0, 8.0, 1.0);
    slp_core::LineParams neg_spread_fit =
        slp_core::RansacFitCore(std::span<const slp_core::Point2D>{normal_pts}, cfg, 12345ULL);
    EXPECT_TRUE(neg_spread_fit.valid);  // 在负 spread 下仍能安全执行无 UB

    // 全超距点（内点数不足）
    cfg.ransac_inlier_threshold = 0.001;
    EXPECT_FALSE(slp_core::IsLineGoodCore(slp_core::LineParams(0.0, 0.0),
                                          std::span<const slp_core::Point2D>{normal_pts}, cfg.ransac_inlier_threshold,
                                          cfg));
}

// ── 3. 参数守卫与可证伪性（Hough 参数防 UB）─────────────────────────────────

TEST(LineFitCoreTest, HoughMinInlierRatioNanAndInfGuards) {
    auto pts = MakeLinePoints(0.05, 1.0, 1.0, 10.0, 1.0);
    slp_core::LineDetectorConfig cfg;
    cfg.enable_hough = true;  // 显式使能 Hough，防止前置未通过导致假绿

    // (1) NaN 守卫：修复前在此触发 static_cast<int>(NaN) 的 UB
    cfg.hough_min_inlier_ratio = std::numeric_limits<double>::quiet_NaN();
    slp_core::LineParams nan_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_FALSE(nan_res.valid);

    // (2) +inf 守卫
    cfg.hough_min_inlier_ratio = std::numeric_limits<double>::infinity();
    slp_core::LineParams inf_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_FALSE(inf_res.valid);

    // (3) -inf 守卫
    cfg.hough_min_inlier_ratio = -std::numeric_limits<double>::infinity();
    slp_core::LineParams neg_inf_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_FALSE(neg_inf_res.valid);

    // (4) 负比例守卫
    cfg.hough_min_inlier_ratio = -0.5;
    slp_core::LineParams neg_ratio_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_FALSE(neg_ratio_res.valid);
}

TEST(LineFitCoreTest, HoughMinInlierRatioHugeFiniteValueGuard) {
    auto pts = MakeLinePoints(0.05, 1.0, 1.0, 10.0, 1.0);
    slp_core::LineDetectorConfig cfg;
    cfg.enable_hough = true;  // 显式使能 Hough，防止前置未通过导致假绿

    // (1) 1e300：#78 的原始复现值。修复前在 line_fit_core.hpp 的
    //     static_cast<int>(ceil(ratio * cones.size())) 处触发 float→int 溢出 UB
    //     （UBSan: 1e+301 is outside the range of representable values of type 'int'）
    cfg.hough_min_inlier_ratio = 1e300;
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    // (2) INT_MAX 附近：乘积仍是有限 double，但超出 int 可表示范围
    cfg.hough_min_inlier_ratio = 1e9;
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    // (3) double 最大值：ratio * 锥桶数 溢出成 inf，同样不得触发 UB
    cfg.hough_min_inlier_ratio = std::numeric_limits<double>::max();
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    // (4) 刚越过上界 1.0：比例语义上界，判非法（与旧行为等价：阈值本就大于总票数）
    cfg.hough_min_inlier_ratio = 1.0 + 1e-9;
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    // (5) 边界仍合法：ratio = 0.0 与 1.0 的输出必须与修复前逐字一致。
    //     逐字一致性由 build/repro78.cpp 的黄金快照 diff 单独证明（修复前后位模式相同）；
    //     此处断言的是「边界值仍被接受、且两端取到同一峰值」这一语义不变量。
    //     注：rho 显式取 0.05（细分辨率）——默认 0.30 下 Hough 离散化误差达 0.067，
    //     属算法本身的分辨率特性，与本守卫无关，不能用它做期望值基准。
    cfg.hough_rho_resolution = 0.05;
    cfg.hough_min_inlier_ratio = 0.0;
    const auto zero_ratio = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_TRUE(zero_ratio.valid);
    EXPECT_NEAR(zero_ratio.slope, 0.05, 0.05);
    EXPECT_NEAR(zero_ratio.intercept, 1.0, 0.2);

    cfg.hough_min_inlier_ratio = 1.0;
    const auto one_ratio = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_TRUE(one_ratio.valid);
    EXPECT_NEAR(one_ratio.slope, 0.05, 0.05);
    EXPECT_NEAR(one_ratio.intercept, 1.0, 0.2);

    // ratio = 0 与 ratio = 1 的自适应阈值同为总票数，取到同一峰值 → 输出逐位相同
    EXPECT_DOUBLE_EQ(zero_ratio.slope, one_ratio.slope);
    EXPECT_DOUBLE_EQ(zero_ratio.intercept, one_ratio.intercept);
}

TEST(LineFitCoreTest, HoughRhoResolutionDegradationDefined) {
    auto pts = MakeLinePoints(0.05, 1.0, 1.0, 10.0, 1.0);
    slp_core::LineDetectorConfig cfg;
    cfg.enable_hough = true;
    cfg.hough_min_inlier_ratio = 0.5;

    // (1) +inf 与极大值（1e300）：静默退化有明确定义，不崩溃、不分配异常大/负数组
    cfg.hough_rho_resolution = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    cfg.hough_rho_resolution = 1e300;
    EXPECT_FALSE(slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg).valid);

    // (2) 负数 / 零 / NaN：仍由原有 std::max(0.05, ·) 钳制为 0.05 正常工作
    cfg.hough_rho_resolution = -1.0;
    slp_core::LineParams neg_rho_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_TRUE(neg_rho_res.valid);
    EXPECT_NEAR(neg_rho_res.slope, 0.05, 0.05);

    cfg.hough_rho_resolution = 0.0;
    slp_core::LineParams zero_rho_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_TRUE(zero_rho_res.valid);

    cfg.hough_rho_resolution = std::numeric_limits<double>::quiet_NaN();
    slp_core::LineParams nan_rho_res = slp_core::HoughFitCore(std::span<const slp_core::Point2D>{pts}, cfg);
    EXPECT_TRUE(nan_rho_res.valid);

    // (3) RANSAC 迭代次数为 0 或负数
    cfg.ransac_max_iter = 0;
    EXPECT_FALSE(slp_core::RansacFitCore(std::span<const slp_core::Point2D>{pts}, cfg, 42ULL).valid);
    cfg.ransac_max_iter = -10;
    EXPECT_FALSE(slp_core::RansacFitCore(std::span<const slp_core::Point2D>{pts}, cfg, 42ULL).valid);
}

// ── 4. RANSAC 种子确定性与可复现性 ──────────────────────────────────────────

TEST(LineFitCoreTest, RansacDeterministicWithInjectedSeed) {
    auto pts = MakeLinePoints(0.12, -0.8, 1.0, 12.0, 0.8);
    // 加入微小噪声测试采样稳定性
    pts.push_back(slp_core::Point2D{5.0, 0.12 * 5.0 - 0.8 + 0.05});
    pts.push_back(slp_core::Point2D{8.0, 0.12 * 8.0 - 0.8 - 0.04});

    slp_core::LineDetectorConfig cfg;
    cfg.ransac_max_iter = 100;
    constexpr std::uint64_t kSeed = 20261019ULL;

    slp_core::LineParams run1 = slp_core::RansacFitCore(std::span<const slp_core::Point2D>{pts}, cfg, kSeed);
    slp_core::LineParams run2 = slp_core::RansacFitCore(std::span<const slp_core::Point2D>{pts}, cfg, kSeed);

    EXPECT_TRUE(run1.valid);
    EXPECT_TRUE(run2.valid);
    // 两次运行逐位完全相同
    EXPECT_DOUBLE_EQ(run1.slope, run2.slope);
    EXPECT_DOUBLE_EQ(run1.intercept, run2.intercept);
}

// ── 5. 时序滤波状态机与跳转抑制 ──────────────────────────────────────────────

TEST(LineFitCoreTest, TemporalFilterTransitions) {
    slp_core::LineDetectorConfig cfg;
    cfg.enable_temporal_filter = true;
    cfg.temporal_filter_alpha = 0.3;
    cfg.temporal_filter_jump_threshold = 0.4;
    cfg.temporal_filter_intercept_jump = 4.0;
    cfg.prev_max_age_frames = 3;

    slp_core::LineParams prev;
    bool has_prev = false;
    int prev_age = 0;

    // (1) 滤波关闭直通
    slp_core::LineDetectorConfig disabled_cfg = cfg;
    disabled_cfg.enable_temporal_filter = false;
    slp_core::LineParams line1(0.1, 1.0);
    slp_core::LineParams f1 = slp_core::ApplyTemporalFilterCore(line1, prev, has_prev, prev_age, disabled_cfg);
    EXPECT_DOUBLE_EQ(f1.slope, 0.1);
    EXPECT_DOUBLE_EQ(f1.intercept, 1.0);

    // 重置状态
    has_prev = false;
    prev_age = 0;

    // (2) 首帧直通
    slp_core::LineParams f2 = slp_core::ApplyTemporalFilterCore(line1, prev, has_prev, prev_age, cfg);
    EXPECT_TRUE(has_prev);
    EXPECT_EQ(prev_age, 0);
    EXPECT_DOUBLE_EQ(f2.slope, 0.1);
    EXPECT_DOUBLE_EQ(f2.intercept, 1.0);

    // (3) 正常小变化指数平滑
    slp_core::LineParams line2(0.2, 1.2);
    slp_core::LineParams f3 = slp_core::ApplyTemporalFilterCore(line2, prev, has_prev, prev_age, cfg);
    // alpha = 0.3: 0.3 * 0.2 + 0.7 * 0.1 = 0.06 + 0.07 = 0.13
    EXPECT_NEAR(f3.slope, 0.13, 1e-6);
    EXPECT_NEAR(f3.intercept, 0.3 * 1.2 + 0.7 * 1.0, 1e-6);
    EXPECT_EQ(prev_age, 0);

    // (4) 突变拒绝（jump 拒绝），续用上一有效值
    slp_core::LineParams jump_line(0.8, 1.2);  // slope 差 0.8 - 0.13 = 0.67 > 0.4
    slp_core::LineParams f4 = slp_core::ApplyTemporalFilterCore(jump_line, prev, has_prev, prev_age, cfg);
    EXPECT_DOUBLE_EQ(f4.slope, f3.slope);
    EXPECT_EQ(prev_age, 1);

    // 连续突变，第 2 帧继续拒绝
    slp_core::LineParams f5 = slp_core::ApplyTemporalFilterCore(jump_line, prev, has_prev, prev_age, cfg);
    EXPECT_DOUBLE_EQ(f5.slope, f3.slope);
    EXPECT_EQ(prev_age, 2);

    // 第 3 帧继续拒绝（prev_max_age_frames = 3）
    slp_core::LineParams f6 = slp_core::ApplyTemporalFilterCore(jump_line, prev, has_prev, prev_age, cfg);
    EXPECT_DOUBLE_EQ(f6.slope, f3.slope);
    EXPECT_EQ(prev_age, 3);

    // 第 4 帧达到超龄限制，强制接受新值，避免陷入停滞
    slp_core::LineParams f7 = slp_core::ApplyTemporalFilterCore(jump_line, prev, has_prev, prev_age, cfg);
    EXPECT_DOUBLE_EQ(f7.slope, 0.8);
    EXPECT_EQ(prev_age, 0);
}

// ── 6. 鸭子类型（Position2DLike）多类型适配 ─────────────────────────────────

TEST(LineFitCoreTest, DuckTypingCustomConeCompatibility) {
    slp_core::LineDetectorConfig cfg;
    std::vector<CustomBaseLinkCone> custom_cones;
    for (int i = 0; i < 8; ++i) {
        CustomBaseLinkCone c;
        c.position_base_link.x = static_cast<double>(i) + 1.0;
        c.position_base_link.y = 0.1 * (static_cast<double>(i) + 1.0) + 0.5;
        c.position_base_link.z = 0.0;
        custom_cones.push_back(c);
    }

    // 直接对 CustomBaseLinkCone 执行 LSQ 拟合
    slp_core::LineParams fit = slp_core::LeastSquaresFitCore(std::span<const CustomBaseLinkCone>{custom_cones}, cfg);
    EXPECT_TRUE(fit.valid);
    EXPECT_NEAR(fit.slope, 0.1, 1e-4);
    EXPECT_NEAR(fit.intercept, 0.5, 1e-4);

    // 计算点到直线距离
    double dist = slp_core::PointToLineDistanceCore(custom_cones[0], fit.slope, fit.intercept);
    EXPECT_NEAR(dist, 0.0, 1e-4);
}
