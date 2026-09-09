#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "vehicle_simulator/bicycle_model.hpp"
#include "vehicle_simulator/sensor_simulator.hpp"

using namespace simulation;

constexpr double kPi = std::numbers::pi_v<double>;

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

TEST(SensorSimulatorTest, FOVAndDistanceFiltering) {
    SensorSimulator sim;
    std::vector<TrackCone> cones = {
        TrackCone{.x = 5.0, .y = 0.0, .type = 0, .id = 1},   // 正前方 5m: 应可见
        TrackCone{.x = 10.0, .y = 2.0, .type = 1, .id = 2},  // 前方偏左在 120 度内: 应可见
        TrackCone{.x = -5.0, .y = 0.0, .type = 0, .id = 3},  // 车身正后方: 应过滤
        TrackCone{.x = 25.0, .y = 0.0, .type = 1, .id = 4},  // 超出 15m 测距: 应过滤
        TrackCone{.x = 2.0, .y = 10.0, .type = 0, .id = 5}   // 偏角超过 60 度 (FOV/2): 应过滤
    };
    sim.SetTrackCones(cones);

    VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    auto detected = sim.GeneratePerceivedCones(car, 120.0, 15.0, 0.0);

    EXPECT_EQ(detected.cone.size(), 2u);

    // 验证探测到的坐标正确映射到 base_link
    for (const auto& c : detected.cone) {
        if (c.id == 1) {
            EXPECT_NEAR(c.position_base_link.x, 5.0f, 1e-3);
            EXPECT_NEAR(c.position_base_link.y, 0.0f, 1e-3);
        } else if (c.id == 2) {
            EXPECT_NEAR(c.position_base_link.x, 10.0f, 1e-3);
            EXPECT_NEAR(c.position_base_link.y, 2.0f, 1e-3);
        }
    }
}
