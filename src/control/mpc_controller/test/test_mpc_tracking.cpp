#include <gtest/gtest.h>

#include "mpc_controller/mpc_model.hpp"

using namespace mpc;

TEST(MpcModelTest, StraightLineLateralErrorCorrection) {
    MpcConfig config;
    config.system.wheelbase = 1.53;
    config.horizon.Np = 15;
    config.horizon.Nc = 10;
    config.horizon.Ts = 0.05;
    config.horizon.target_speed = 8.0;

    MpcModel model(config);

    // 沿 X 轴的直线参考路径
    std::vector<ReferencePoint> path;
    for (double x = 0.0; x <= 50.0; x += 0.5) {
        path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = 8.0});
    }

    // 车辆当前在 y = 0.4m (左偏)，期望向右打方向 (转角为负)
    auto sol_left = model.Step(5.0, 0.4, 0.0, 8.0, 0.0, 0.0, path);
    EXPECT_TRUE(sol_left.success);
    EXPECT_LT(sol_left.steering_rad, -0.01);  // 应向右打方向纠偏

    // 车辆当前在 y = -0.4m (右偏)，期望向左打方向 (转角为正)
    auto sol_right = model.Step(5.0, -0.4, 0.0, 8.0, 0.0, 0.0, path);
    EXPECT_TRUE(sol_right.success);
    EXPECT_GT(sol_right.steering_rad, 0.01);  // 应向左打方向纠偏
}

TEST(MpcModelTest, PerfectTrackingNearZeroInputs) {
    MpcConfig config;
    MpcModel model(config);

    std::vector<ReferencePoint> path;
    for (double x = 0.0; x <= 30.0; x += 0.5) {
        path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = 8.0});
    }

    // 车辆完全处于参考路径上，且速度达到目标
    auto sol = model.Step(10.0, 0.0, 0.0, 8.0, 0.0, 0.0, path);
    EXPECT_TRUE(sol.success);
    EXPECT_NEAR(sol.steering_rad, 0.0, 0.02);
    EXPECT_NEAR(sol.accel_mps2, 0.0, 0.1);
    EXPECT_GT(sol.predicted_trajectory.size(), 10u);
}

TEST(MpcModelTest, SpeedTrackingAccelerationAndBraking) {
    MpcConfig config;
    config.horizon.target_speed = 10.0;
    MpcModel model(config);

    std::vector<ReferencePoint> path;
    for (double x = 0.0; x <= 30.0; x += 0.5) {
        path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = 10.0});
    }

    // 1. 当前车速 5m/s < 10m/s，期望正加速度加速
    auto sol_accel = model.Step(5.0, 0.0, 0.0, 5.0, 0.0, 0.0, path);
    EXPECT_TRUE(sol_accel.success);
    EXPECT_GT(sol_accel.accel_mps2, 0.2);

    // 2. 当前车速 15m/s > 10m/s，期望负加速度制动减速
    auto sol_brake = model.Step(5.0, 0.0, 0.0, 15.0, 0.0, 0.0, path);
    EXPECT_TRUE(sol_brake.success);
    EXPECT_LT(sol_brake.accel_mps2, -0.2);
}

TEST(MpcModelTest, ExplicitZeroSpeedRemainsAStopTarget) {
    MpcConfig config;
    MpcModel model(config);

    std::vector<ReferencePoint> path;
    for (double x = 0.0; x <= 30.0; x += 0.5) {
        path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = 0.0, .speed_valid = true});
    }

    const auto solution = model.Step(5.0, 0.0, 0.0, 5.0, 0.0, 0.0, path);

    ASSERT_TRUE(solution.success);
    EXPECT_LT(solution.accel_mps2, -0.2);
}

// issue #3：同一几何轨迹分别以 map（非零平移+非零航向）与 base_link（局部契约）输入，控制解应等价
TEST(MpcModelTest, GlobalAndLocalFrameInputsYieldEquivalentControls) {
    MpcConfig config;
    config.system.wheelbase = 1.53;
    config.horizon.Np = 15;
    config.horizon.Nc = 10;
    config.horizon.Ts = 0.05;
    config.horizon.target_speed = 8.0;
    MpcModel model(config);

    const double theta0 = 0.7;  // 非零全局航向
    const double tx = 100.0, ty = 50.0;
    const double c = std::cos(theta0), s = std::sin(theta0);

    std::vector<ReferencePoint> local_path;
    std::vector<ReferencePoint> global_path;
    for (double x = 0.0; x <= 50.0; x += 0.5) {
        // 局部系：沿 X 轴直线（已知可收敛场景），全局系：同一轨迹经非零平移+旋转
        local_path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = 8.0, .speed_valid = true});
        global_path.push_back(
            {.x = tx + c * x, .y = ty + s * x, .theta = theta0, .curvature = 0.0, .speed = 8.0, .speed_valid = true});
    }

    // 同一物理场景：车辆在局部系 (5, 0.4, 0) ⇔ 全局系 T + R(θ0)·(5, 0.4)，航向 θ0
    const auto sol_local = model.Step(5.0, 0.4, 0.0, 8.0, 0.0, 0.0, local_path);
    const auto sol_global =
        model.Step(tx + c * 5.0 - s * 0.4, ty + s * 5.0 + c * 0.4, theta0, 8.0, 0.0, 0.0, global_path);

    ASSERT_TRUE(sol_local.success);
    ASSERT_TRUE(sol_global.success);
    EXPECT_NEAR(sol_global.steering_rad, sol_local.steering_rad, 1e-3);
    EXPECT_NEAR(sol_global.accel_mps2, sol_local.accel_mps2, 1e-3);
    EXPECT_LT(sol_local.steering_rad, -0.01);  // 与既有纠偏方向断言一致（左偏→右打）
}
