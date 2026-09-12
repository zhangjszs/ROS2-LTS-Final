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
