// #53：本文件现只保留 SensorSimulatorTest（依赖 common_msgs/msgs，colcon 侧运行）。
// BicycleModel 的纯 std 用例已移入 test_bicycle_model_core.cpp，并在
// tests/core_standalone 注册以进入 #40 的 ASan+UBSan 门。
#include <gtest/gtest.h>

#include <vector>

#include "vehicle_simulator/sensor_simulator.hpp"

using namespace simulation;

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
