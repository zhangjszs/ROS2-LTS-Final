#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

#include "interface_contract.h"  // #14：验证剖面形成合法的 target_speeds 速度载体
#include "velocity_profiler/velocity_profiler.hpp"

namespace velocity_profiler {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;

}  // namespace

TEST(VelocityProfilerTest, StraightLineAcceleration) {
    ProfilerLimits limits;
    limits.max_velocity = 20.0;
    limits.min_velocity = 2.0;
    limits.max_lon_accel = 4.0;
    limits.max_lat_accel = 9.8;

    VelocityProfiler profiler(limits);

    // 100 米直线，步长 1 米
    std::vector<std::pair<double, double>> path;
    for (double x = 0.0; x <= 100.0; x += 1.0) {
        path.emplace_back(x, 0.0);
    }

    auto profile = profiler.ComputeProfile(path, 0.0);

    ASSERT_EQ(profile.size(), path.size());
    EXPECT_NEAR(profile.front().target_speed, limits.min_velocity, 1e-2);
    EXPECT_NEAR(profile.back().target_speed, limits.max_velocity, 1e-2);

    // 验证加速度不超过限制
    for (size_t i = 1; i < profile.size(); ++i) {
        double v_prev = profile[i - 1].target_speed;
        double v_curr = profile[i].target_speed;
        double ds = profile[i].s - profile[i - 1].s;
        double accel = (v_curr * v_curr - v_prev * v_prev) / (2.0 * ds);
        EXPECT_LE(accel, limits.max_lon_accel + 1e-3);
    }
}

TEST(VelocityProfilerTest, CornerDecelerationAndSpeedLimit) {
    ProfilerLimits limits;
    limits.max_velocity = 20.0;
    limits.min_velocity = 2.0;
    limits.max_lat_accel = 9.8;
    limits.max_lon_decel = 5.0;
    limits.curvature_smoothing_window = 3;

    VelocityProfiler profiler(limits);

    // 1. 直线 40 米
    std::vector<std::pair<double, double>> path;
    for (double x = 0.0; x <= 40.0; x += 0.5) {
        path.emplace_back(x, 0.0);
    }

    // 2. 接半径 R = 9.125 的圆弧 (Skidpad 圆)
    const double r = 9.125;
    const double cx = 40.0;
    const double cy = r;
    for (double deg = -90.0; deg <= 90.0; deg += 3.0) {
        double rad = deg * (kPi / 180.0);
        path.emplace_back(cx + r * std::cos(rad), cy + r * std::sin(rad));
    }

    // 假设初始高速 18 m/s
    auto profile = profiler.ComputeProfile(path, 18.0);
    ASSERT_EQ(profile.size(), path.size());

    // 理论弯道限速: v = sqrt(a_y_max * R) = sqrt(9.8 * 9.125) = 9.456 m/s
    const double theoretical_corner_v = std::sqrt(limits.max_lat_accel * r);

    // 验证弯道内部点限速符合向心力公式
    size_t corner_start_idx = 81;  // x=40 之后的点
    for (size_t i = corner_start_idx + 10; i < profile.size() - 5; ++i) {
        EXPECT_LE(profile[i].target_speed, theoretical_corner_v + 0.8);
    }

    // 验证直道末尾在进弯前已提前减速 (后向制动滤波生效)
    EXPECT_LT(profile[corner_start_idx].target_speed, 12.0);
}

TEST(VelocityProfilerTest, CornerLimitOverridesMinimumCruisingSpeed) {
    ProfilerLimits limits;
    limits.min_velocity = 3.0;
    limits.max_velocity = 20.0;
    limits.max_lat_accel = 1.0;
    limits.curvature_smoothing_window = 1;

    VelocityProfiler profiler(limits);
    std::vector<std::pair<double, double>> path;
    constexpr double radius = 5.0;
    for (int i = 0; i < 100; ++i) {
        const double angle = static_cast<double>(i) * 0.03;
        path.emplace_back(radius * std::sin(angle), radius * (1.0 - std::cos(angle)));
    }

    const auto profile = profiler.ComputeProfile(path, 3.0);
    const double expected_max_speed = std::sqrt(limits.max_lat_accel * radius);
    bool produced_below_minimum = false;
    for (const auto& point : profile) {
        EXPECT_LE(point.target_speed, expected_max_speed + 0.05);
        EXPECT_LE(point.target_speed * point.target_speed * std::abs(point.curvature), limits.max_lat_accel + 0.05);
        produced_below_minimum = produced_below_minimum || point.target_speed < limits.min_velocity;
    }
    EXPECT_TRUE(produced_below_minimum);
}

TEST(VelocityProfilerTest, EmptyAndDegeneratePath) {
    VelocityProfiler profiler;

    // 空路径
    auto p0 = profiler.ComputeProfile({});
    EXPECT_TRUE(p0.empty());

    // 单点路径
    auto p1 = profiler.ComputeProfile({{10.0, 20.0}});
    ASSERT_EQ(p1.size(), 1u);
    EXPECT_DOUBLE_EQ(p1[0].x, 10.0);
    EXPECT_DOUBLE_EQ(p1[0].y, 20.0);
}

// #24 必测矩阵第 2 行（畸形轨迹）/ 第 7 行（最低速度与物理约束冲突）的差分夹具。
// ROS1-LTS-Final #9（空/短路径仍刷新心跳）与 #13（最低速度覆盖横向约束）在 ROS2 的具体形态：
// 几何退化为 0/1 点时，速度权威不得发一个“可行驶”速度（旧实现发 min_velocity）。
// 分类：缺陷修复差异（ROS1 行为=给巡航速度；ROS2 期望=明确不可行/停车）。
TEST(DefectDifferential, DegenerateGeometryMustNotYieldDrivableSpeed) {
    ProfilerLimits limits;
    limits.min_velocity = 2.0;
    limits.max_velocity = 20.0;
    VelocityProfiler profiler(limits);

    for (std::vector<std::pair<double, double>> bad : {std::vector<std::pair<double, double>>{},
                                                       {{10.0, 20.0}},
                                                       {{10.0, std::numeric_limits<double>::quiet_NaN()}}}) {
        const auto profile = profiler.ComputeProfile(bad, /*current_speed=*/0.0);
        for (const auto& point : profile) {
            // 0.0 = #11 语义下的合法“停车目标”；任何 >0 的速度都等于对畸形几何臆判可行驶
            EXPECT_DOUBLE_EQ(point.target_speed, 0.0) << "input points=" << bad.size();
        }
    }

    // 与契约层一致：1 点几何本身即不合法，消费者不得把它当可用轨迹（#14 §2）。
    const std::vector<double> xs{10.0};
    const std::vector<double> ys{20.0};
    EXPECT_FALSE(common_msgs::contract::geometryValid(xs, ys));
}

// #14 项②：作为速度权威，velocity_profiler 输出的 target_speeds 必须是下游
// 契约门 (targetSpeedsEffective) 接受的合法载体：与路径等长、逐点有限且 >=0（允许 0）。
TEST(VelocityProfilerTest, ProfileFormsValidTargetSpeedsCarrier) {
    ProfilerLimits limits;
    limits.max_velocity = 20.0;
    limits.min_velocity = 2.0;
    limits.max_lat_accel = 9.8;
    VelocityProfiler profiler(limits);

    std::vector<std::pair<double, double>> path;
    for (double x = 0.0; x <= 50.0; x += 1.0) {
        path.emplace_back(x, 0.0);
    }

    const auto profile = profiler.ComputeProfile(path, 0.0);
    ASSERT_EQ(profile.size(), path.size());

    // 将剖面展开为节点将写入 HuatPathLimits.target_speeds 的 double 数组
    std::vector<double> speeds;
    speeds.reserve(profile.size());
    for (const auto& pp : profile) {
        speeds.push_back(pp.target_speed);
    }

    // 下游消费者据此判定是否采用 target_speeds（而非回退默认/绝不应从 z 取速）
    EXPECT_TRUE(common_msgs::contract::targetSpeedsEffective(speeds, path.size()));
}

}  // namespace velocity_profiler
