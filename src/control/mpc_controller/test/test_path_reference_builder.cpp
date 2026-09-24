// issue #22：mpc_core::BuildReferencePath / SelectReferenceOrigin 单元测试
// —— 从 MPC 节点 OnPath 抽出的 path→ReferencePoint 纯算法（无 ROS context）。
// 覆盖：直线/曲线数值、target_speeds 缺失与非法回退、显式零速、frame 门禁、
// 点数不足、base_link vs map 一致性、NaN 输入确定性传播、重复点不除零。
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>
#include <vector>

#include "mpc_controller/mpc_types.hpp"
#include "mpc_controller/path_reference_builder.h"

namespace {

// 与 geometry_msgs::msg::Point 同形的最小点类型：证明 core 不依赖 ROS 消息即可单测。
struct Pt {
    double x{0.0};
    double y{0.0};
};

static_assert(mpc_core::PlanarPointLike<Pt>, "Pt 必须满足平面点约束");
static_assert(!mpc_core::PlanarPointLike<double>, "非点类型不得满足平面点约束");

// 沿 X 轴等距直线：第 i 点为 (i*step, 0)。
std::vector<Pt> MakeLine(std::size_t count, double step) {
    std::vector<Pt> pts;
    pts.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        pts.emplace_back(Pt{static_cast<double>(i) * step, 0.0});
    }
    return pts;
}

// 半径 radius 的圆弧（圆心 (0, radius)，自原点沿逆时针展开），相邻点角度步长 dphi。
std::vector<Pt> MakeArc(std::size_t count, double radius, double dphi) {
    std::vector<Pt> pts;
    pts.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double phi = static_cast<double>(i) * dphi;
        pts.emplace_back(Pt{radius * std::sin(phi), radius * (1.0 - std::cos(phi))});
    }
    return pts;
}

mpc_core::ReferencePathResult Build(std::string_view frame, const std::vector<Pt>& pts,
                                    const std::vector<double>& speeds, double default_speed) {
    return mpc_core::BuildReferencePath(frame, std::span<const Pt>{pts}, std::span<const double>{speeds},
                                        default_speed);
}

constexpr double kDefaultSpeed = 8.0;

}  // namespace

TEST(PathReferenceBuilder, StraightLineFillsHeadingsAndSpeeds) {
    const auto pts = MakeLine(4, 1.0);
    const std::vector<double> speeds = {4.0, 4.0, 4.0, 4.0};

    const auto r = Build("map", pts, speeds, 0.0);

    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    EXPECT_FALSE(r.in_base_frame);
    ASSERT_EQ(r.points.size(), 4u);
    for (std::size_t i = 0; i < r.points.size(); ++i) {
        EXPECT_DOUBLE_EQ(r.points[i].x, static_cast<double>(i));
        EXPECT_DOUBLE_EQ(r.points[i].y, 0.0);
        EXPECT_DOUBLE_EQ(r.points[i].theta, 0.0);      // 沿 +X 直线
        EXPECT_DOUBLE_EQ(r.points[i].curvature, 0.0);  // ds>1e-3 但 dtheta=0
        EXPECT_DOUBLE_EQ(r.points[i].speed, 4.0);      // 显式速度优先于回退值
        EXPECT_TRUE(r.points[i].speed_valid);
    }
}

TEST(PathReferenceBuilder, ArcEstimatesHeadingAndCurvature) {
    constexpr double kRadius = 5.0;
    constexpr double kDphi = 0.05;
    const auto pts = MakeArc(21, kRadius, kDphi);
    const std::vector<double> speeds(pts.size(), 6.0);

    const auto r = Build("map", pts, speeds, 0.0);

    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    ASSERT_EQ(r.points.size(), pts.size());
    EXPECT_DOUBLE_EQ(r.points.front().curvature, 0.0);                        // 首点无前向差分
    EXPECT_DOUBLE_EQ(r.points.back().curvature, 0.0);                         // 末点无后向差分
    EXPECT_DOUBLE_EQ(r.points.back().theta, r.points[pts.size() - 2].theta);  // 末点继承前点航向

    for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
        const double expected_theta = std::atan2(pts[i + 1].y - pts[i].y, pts[i + 1].x - pts[i].x);
        EXPECT_DOUBLE_EQ(r.points[i].theta, expected_theta);
        const double dtheta = mpc::NormalizeAngle(r.points[i].theta - r.points[i - 1].theta);
        const double ds = std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
        EXPECT_DOUBLE_EQ(r.points[i].curvature, dtheta / ds);
        EXPECT_NEAR(r.points[i].curvature, 1.0 / kRadius, 1e-3);  // 弦差分离散近似 ≈ 1/R
    }
}

TEST(PathReferenceBuilder, MissingTargetSpeedsFallBackToDefault) {
    const auto pts = MakeLine(3, 1.0);
    const std::vector<double> none;

    const auto r = Build("map", pts, none, 3.0);
    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    for (const auto& pt : r.points) {
        EXPECT_DOUBLE_EQ(pt.speed, 3.0);
        EXPECT_TRUE(pt.speed_valid);
    }

    // 长度与路径不等同样视为缺失（契约：等长才可信）
    const std::vector<double> too_short = {1.0, 2.0};
    const auto r2 = Build("map", pts, too_short, 3.0);
    ASSERT_EQ(r2.status, mpc_core::ReferencePathStatus::Ok);
    for (const auto& pt : r2.points) {
        EXPECT_DOUBLE_EQ(pt.speed, 3.0);
        EXPECT_TRUE(pt.speed_valid);
    }
}

TEST(PathReferenceBuilder, InvalidTargetSpeedsFallBackToDefault) {
    const auto pts = MakeLine(3, 1.0);

    const std::vector<double> with_nan = {1.0, std::nan(""), 1.0};
    for (const auto& pt : Build("map", pts, with_nan, 2.0).points) {
        EXPECT_DOUBLE_EQ(pt.speed, 2.0);
        EXPECT_TRUE(pt.speed_valid);
    }

    const std::vector<double> with_negative = {1.0, -1.0, 1.0};
    for (const auto& pt : Build("map", pts, with_negative, 2.0).points) {
        EXPECT_DOUBLE_EQ(pt.speed, 2.0);
        EXPECT_TRUE(pt.speed_valid);
    }
}

TEST(PathReferenceBuilder, ExplicitZeroSpeedIsValidStopTarget) {
    const auto pts = MakeLine(3, 1.0);
    const std::vector<double> zeros = {0.0, 0.0, 0.0};

    const auto r = Build("map", pts, zeros, kDefaultSpeed);  // 显式 0 不得被默认速度覆盖

    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    for (const auto& pt : r.points) {
        EXPECT_DOUBLE_EQ(pt.speed, 0.0);
        EXPECT_TRUE(pt.speed_valid);
    }
}

TEST(PathReferenceBuilder, DefaultSpeedThresholdMarksValidity) {
    const auto pts = MakeLine(2, 1.0);
    const std::vector<double> none;

    // >0.1 视为可行驶
    EXPECT_TRUE(Build("map", pts, none, 0.2).points[0].speed_valid);
    // <=0.1 视为不可行驶（降级），但速度值仍然透传
    const auto low = Build("map", pts, none, 0.05);
    EXPECT_DOUBLE_EQ(low.points[0].speed, 0.05);
    EXPECT_FALSE(low.points[0].speed_valid);
    EXPECT_FALSE(Build("map", pts, none, 0.0).points[0].speed_valid);
    // 非有限默认值：NaN 透传 + 标记无效
    const auto nan_default = Build("map", pts, none, std::nan(""));
    EXPECT_TRUE(std::isnan(nan_default.points[0].speed));
    EXPECT_FALSE(nan_default.points[0].speed_valid);
}

TEST(PathReferenceBuilder, RejectsUnsupportedFrame) {
    const auto pts = MakeLine(3, 1.0);
    const std::vector<double> speeds(pts.size(), 5.0);

    for (const std::string_view frame : {"odom", "", "BASE_LINK"}) {  // 精确匹配：大小写敏感
        const auto r = Build(frame, pts, speeds, kDefaultSpeed);
        EXPECT_EQ(r.status, mpc_core::ReferencePathStatus::RejectedFrame);
        EXPECT_TRUE(r.points.empty());
        EXPECT_FALSE(r.in_base_frame);
    }
}

TEST(PathReferenceBuilder, TooFewPointsYieldsNoResultButReportsFrame) {
    const std::vector<double> speeds;

    const std::vector<Pt> none;
    const auto r0 = Build("map", none, speeds, kDefaultSpeed);
    EXPECT_EQ(r0.status, mpc_core::ReferencePathStatus::TooFewPoints);
    EXPECT_TRUE(r0.points.empty());
    EXPECT_FALSE(r0.in_base_frame);

    const std::vector<Pt> single = {{1.0, 2.0}};
    const auto r1 = Build("base_link", single, speeds, kDefaultSpeed);
    EXPECT_EQ(r1.status, mpc_core::ReferencePathStatus::TooFewPoints);
    EXPECT_TRUE(r1.points.empty());
    EXPECT_TRUE(r1.in_base_frame);  // frame 合法时仍报告坐标系语义
}

TEST(PathReferenceBuilder, BaseLinkAndMapRepresentationsStayConsistent) {
    constexpr double kTheta0 = 0.7;
    constexpr double kTx = 100.0;
    constexpr double kTy = 50.0;
    const double c = std::cos(kTheta0);
    const double s = std::sin(kTheta0);

    const auto local = MakeLine(11, 1.0);
    std::vector<Pt> global;
    global.reserve(local.size());
    for (const auto& p : local) {  // 同一物理轨迹：R(θ0) 旋转 + 平移 T
        global.emplace_back(Pt{kTx + c * p.x - s * p.y, kTy + s * p.x + c * p.y});
    }
    const std::vector<double> speeds(local.size(), 5.0);

    const auto rl = Build("base_link", local, speeds, 0.0);
    const auto rg = Build("map", global, speeds, 0.0);

    ASSERT_EQ(rl.status, mpc_core::ReferencePathStatus::Ok);
    ASSERT_EQ(rg.status, mpc_core::ReferencePathStatus::Ok);
    EXPECT_TRUE(rl.in_base_frame);
    EXPECT_FALSE(rg.in_base_frame);
    ASSERT_EQ(rl.points.size(), rg.points.size());

    for (std::size_t i = 0; i < rl.points.size(); ++i) {
        // map 表示相对全局航向的转角 == base_link 表示的局部航向（直线均为 0）
        EXPECT_NEAR(rg.points[i].theta - kTheta0, rl.points[i].theta, 1e-12);
        EXPECT_NEAR(rg.points[i].curvature, rl.points[i].curvature, 1e-12);
        EXPECT_DOUBLE_EQ(rg.points[i].speed, rl.points[i].speed);
        EXPECT_EQ(rg.points[i].speed_valid, rl.points[i].speed_valid);
        if (i > 0) {  // 相邻点弦长（几何尺度）在两种表示下一致
            EXPECT_NEAR(std::hypot(global[i].x - global[i - 1].x, global[i].y - global[i - 1].y),
                        std::hypot(local[i].x - local[i - 1].x, local[i].y - local[i - 1].y), 1e-12);
        }
    }
}

TEST(PathReferenceBuilder, SelectReferenceOriginFollowsFrameContract) {
    // base_link：车辆视为原点、航向 0（局部路径契约）
    const auto local = mpc_core::SelectReferenceOrigin(true, 12.0, 3.0, 1.1);
    EXPECT_DOUBLE_EQ(local.x, 0.0);
    EXPECT_DOUBLE_EQ(local.y, 0.0);
    EXPECT_DOUBLE_EQ(local.theta, 0.0);

    // map：参考系原点为车辆全局位姿本身（数值原样透传）
    const auto global = mpc_core::SelectReferenceOrigin(false, 12.0, 3.0, 1.1);
    EXPECT_DOUBLE_EQ(global.x, 12.0);
    EXPECT_DOUBLE_EQ(global.y, 3.0);
    EXPECT_DOUBLE_EQ(global.theta, 1.1);
}

TEST(PathReferenceBuilder, NaNGeesometryDegradesDeterministically) {
    const std::vector<Pt> pts = {{0.0, 0.0}, {std::nan(""), 0.0}, {2.0, 0.0}};
    const std::vector<double> speeds = {1.0, 1.0, 1.0};

    const auto r = Build("map", pts, speeds, 0.0);

    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    ASSERT_EQ(r.points.size(), 3u);
    EXPECT_TRUE(std::isnan(r.points[1].x));  // 几何 NaN 原样透传（不静默当 0）
    // 相邻差分含 NaN → 受影响点航向为 NaN，末点继承 NaN（确定性传播，非崩溃/非 UB）
    EXPECT_TRUE(std::isnan(r.points[0].theta));
    EXPECT_TRUE(std::isnan(r.points[1].theta));
    EXPECT_TRUE(std::isnan(r.points[2].theta));
    // ds 为 NaN 时 `ds > 1e-3` 为假 → 曲率退化为 0，不发生除零
    for (const auto& pt : r.points) {
        EXPECT_DOUBLE_EQ(pt.curvature, 0.0);
    }
    // 速度通道不受几何污染（数组逐点有限非负仍显式有效）
    for (const auto& pt : r.points) {
        EXPECT_DOUBLE_EQ(pt.speed, 1.0);
        EXPECT_TRUE(pt.speed_valid);
    }
}

TEST(PathReferenceBuilder, DuplicatePointsYieldZeroCurvature) {
    const std::vector<Pt> pts = {{1.0, 1.0}, {1.0, 1.0}, {2.0, 1.0}};
    const std::vector<double> speeds(pts.size(), 2.0);

    const auto r = Build("map", pts, speeds, 0.0);

    ASSERT_EQ(r.status, mpc_core::ReferencePathStatus::Ok);
    ASSERT_EQ(r.points.size(), 3u);
    EXPECT_DOUBLE_EQ(r.points[0].theta, 0.0);      // atan2(0, 0)
    EXPECT_DOUBLE_EQ(r.points[1].theta, 0.0);      // atan2(0, 1)
    EXPECT_DOUBLE_EQ(r.points[1].curvature, 0.0);  // ds=0 <= 1e-3 → 0（不除零）
}
