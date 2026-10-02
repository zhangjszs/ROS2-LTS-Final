// #53：BicycleModel 纯 std 单测——脱离 ROS/msgs 编译，进 tests/core_standalone 的
// ASan+UBSan 门（#40 约定）。与 colcon 侧注册同一份文件（单一事实来源）。
// 传感器仿真相关的 SensorSimulatorTest 仍留在 test_bicycle_model.cpp（依赖 ROS 生成消息）。
#include <gtest/gtest.h>

#include <cmath>
#include <concepts>

#include "vehicle_simulator/bicycle_model.hpp"

using namespace simulation;

// 1. C++20 Concepts 编译期静态断言约束检查
static_assert(requires(BicycleModel m, const VehicleState& s, const ControlCommand& u, const VehicleParams& p,
                       double dt) {
    { m.StepRK4(s, u, dt) } -> std::same_as<VehicleState>;
    { m.StepEuler(s, u, dt) } -> std::same_as<VehicleState>;
});

TEST(BicycleModelTest, StraightLineAcceleration) {
    VehicleParams params;
    params.max_accel = 5.0;
    BicycleModel model(params);
    model.Reset(0.0, 0.0, 0.0, 0.0);

    ControlCommand cmd{.target_steering = 0.0, .target_accel = 4.0};

    // 运行 1 秒 (100 步, dt = 0.01)
    const double dt = 0.01;
    for (int i = 0; i < 100; ++i) {
        model.Step(cmd, dt);
    }

    const auto& s = model.state();
    EXPECT_GT(s.v, 1.0);                       // 速度应显著提升
    EXPECT_GT(s.x, 0.5);                       // X 坐标向前推进
    EXPECT_NEAR(s.y, 0.0, 1e-4);               // 直线行驶无横向漂移
    EXPECT_NEAR(s.theta, 0.0, 1e-4);           // 航向角保持 0
    EXPECT_NEAR(s.steering_angle, 0.0, 1e-4);  // 前轮无转角
}

TEST(BicycleModelTest, LongitudinalDerivativesHaveDistinctUnits) {
    BicycleModel model;
    VehicleState state{.v = 5.0, .accel = 2.0};
    ControlCommand cmd{.target_accel = 4.0};

    const auto derivative = model.ComputeDerivative(state, cmd);

    EXPECT_DOUBLE_EQ(derivative.v, state.accel);
    EXPECT_DOUBLE_EQ(derivative.accel, 20.0);
}

TEST(BicycleModelTest, AccelerationResponseMatchesFirstOrderModel) {
    VehicleParams params;
    params.max_speed = 100.0;
    BicycleModel model(params);
    model.Reset();

    constexpr double target_accel = 4.0;
    constexpr double dt = 0.001;
    constexpr int steps = 1000;
    for (int i = 0; i < steps; ++i) {
        model.Step(ControlCommand{.target_accel = target_accel}, dt);
    }

    const double time = steps * dt;
    const double tau = params.throttle_time_const;
    const double expected_velocity = target_accel * (time - tau * (1.0 - std::exp(-time / tau)));
    const double expected_accel = target_accel * (1.0 - std::exp(-time / tau));

    EXPECT_NEAR(model.state().v, expected_velocity, 1e-4);
    EXPECT_NEAR(model.state().accel, expected_accel, 1e-4);
}

TEST(BicycleModelTest, SteadyStateCurvatureRadius) {
    VehicleParams params;
    params.wheelbase = 1.55;
    params.steer_time_const = 0.001;  // 极小延迟以快速达到稳态
    BicycleModel model(params);

    const double steer = 0.20;  // rad
    model.Reset(0.0, 0.0, 0.0, 5.0);

    ControlCommand cmd{.target_steering = steer, .target_accel = 0.0};

    // 运行 2 秒让转弯进入稳态
    const double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        model.Step(cmd, dt);
    }

    // 理论转弯半径 R = L / tan(delta)
    const double expected_radius = params.wheelbase / std::tan(steer);
    // 理论角速度 omega = v / R
    const double expected_yaw_rate = model.state().v / expected_radius;

    EXPECT_NEAR(model.state().yaw_rate, expected_yaw_rate, 0.05);
}

TEST(BicycleModelTest, SteerRateLimiting) {
    VehicleParams params;
    params.max_steer_rate = 1.0;      // 限制转速 1.0 rad/s
    params.steer_time_const = 0.001;  // 瞬间指令
    BicycleModel model(params);
    model.Reset(0.0, 0.0, 0.0, 0.0);

    ControlCommand cmd{.target_steering = 0.40, .target_accel = 0.0};
    const double dt = 0.10;  // 0.1s
    model.Step(cmd, dt);

    // 0.1s 内变化不应超过 max_steer_rate * dt = 0.1 rad (加微小公差)
    EXPECT_LE(model.state().steering_angle, 0.11);
}
