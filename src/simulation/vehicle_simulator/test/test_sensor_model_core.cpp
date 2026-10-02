// #54：感知几何核（纯 std）单测——脱离 ROS/msgs，进 tests/core_standalone 的 ASan+UBSan 门。
// 几何断言由适配层测试（test_bicycle_model.cpp 的 SensorSimulatorTest）搬移至此。
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <random>
#include <vector>

#include "vehicle_simulator/sensor_model_core.hpp"

using namespace simulation;

namespace {

// 与重构前 SensorSimulatorTest 相同的输入（正前方 5m / 前方偏左 2m / 正后方 / 超距 / 超 FOV）
std::vector<TrackCone> MakeStandardTrack() {
    return {
        TrackCone{.x = 5.0, .y = 0.0, .type = 0, .id = 1},   // 正前方 5m: 应可见
        TrackCone{.x = 10.0, .y = 2.0, .type = 1, .id = 2},  // 前方偏左在 120 度内: 应可见
        TrackCone{.x = -5.0, .y = 0.0, .type = 0, .id = 3},  // 车身正后方: 应过滤
        TrackCone{.x = 25.0, .y = 0.0, .type = 1, .id = 4},  // 超出 15m 测距: 应过滤
        TrackCone{.x = 2.0, .y = 10.0, .type = 0, .id = 5}   // 偏角超过 60 度 (FOV/2): 应过滤
    };
}

}  // namespace

TEST(SensorModelCoreTest, FOVAndDistanceFilteringWithBaseLinkMapping) {
    std::mt19937 rng{42};
    const VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};

    const auto detected = PredictVisibleCones(MakeStandardTrack(), car, 120.0, 15.0, 0.0, rng);

    ASSERT_EQ(detected.size(), 2u);
    for (const auto& c : detected) {
        EXPECT_EQ(c.confidence, 95u);
        if (c.id == 1) {
            EXPECT_NEAR(c.x_base, 5.0f, 1e-3);
            EXPECT_NEAR(c.y_base, 0.0f, 1e-3);
        } else if (c.id == 2) {
            EXPECT_NEAR(c.x_base, 10.0f, 1e-3);
            EXPECT_NEAR(c.y_base, 2.0f, 1e-3);
        } else {
            ADD_FAILURE() << "unexpected visible cone id " << c.id;
        }
    }
}

TEST(SensorModelCoreTest, RotationMapsGlobalAheadToBaseLinkForward) {
    std::mt19937 rng{42};
    // 车头朝 +Y (theta=90°)：全局 +Y 方向的锥桶应落在 base_link 的 +X
    const VehicleState car{.x = 0.0, .y = 0.0, .theta = std::numbers::pi_v<double> / 2.0, .v = 0.0};
    const std::vector<TrackCone> cones = {TrackCone{.x = 0.0, .y = 5.0, .type = 0, .id = 9}};

    const auto detected = PredictVisibleCones(cones, car, 120.0, 15.0, 0.0, rng);

    ASSERT_EQ(detected.size(), 1u);
    EXPECT_NEAR(detected[0].x_base, 5.0f, 1e-3);
    EXPECT_NEAR(detected[0].y_base, 0.0f, 1e-3);
}

TEST(SensorModelCoreTest, NoNoiseLeavesCoordinatesExact) {
    std::mt19937 rng{42};
    const VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    const std::vector<TrackCone> cones = {TrackCone{.x = 5.0, .y = 3.0, .type = 0, .id = 1}};

    const auto detected = PredictVisibleCones(cones, car, 120.0, 15.0, 0.0, rng);

    ASSERT_EQ(detected.size(), 1u);
    EXPECT_FLOAT_EQ(detected[0].x_base, 5.0f);
    EXPECT_FLOAT_EQ(detected[0].y_base, 3.0f);
    EXPECT_FLOAT_EQ(detected[0].x_global, 5.0f);
    EXPECT_FLOAT_EQ(detected[0].y_global, 3.0f);
}

TEST(SensorModelCoreTest, NoiseIsReproducibleForFixedSeed) {
    const VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    std::mt19937 rng_a{42};
    std::mt19937 rng_b{42};
    std::mt19937 rng_c{7};

    const auto a = PredictVisibleCones(MakeStandardTrack(), car, 120.0, 15.0, 0.1, rng_a);
    const auto b = PredictVisibleCones(MakeStandardTrack(), car, 120.0, 15.0, 0.1, rng_b);
    const auto c = PredictVisibleCones(MakeStandardTrack(), car, 120.0, 15.0, 0.1, rng_c);

    ASSERT_EQ(a.size(), b.size());
    ASSERT_EQ(a.size(), c.size());
    bool differs_from_other_seed = false;
    for (size_t i = 0; i < a.size(); ++i) {
        EXPECT_FLOAT_EQ(a[i].x_base, b[i].x_base);
        EXPECT_FLOAT_EQ(a[i].y_base, b[i].y_base);
        EXPECT_FLOAT_EQ(a[i].x_global, b[i].x_global);
        EXPECT_FLOAT_EQ(a[i].y_global, b[i].y_global);
        if (a[i].x_base != c[i].x_base || a[i].y_base != c[i].y_base)
            differs_from_other_seed = true;
    }
    EXPECT_TRUE(differs_from_other_seed);
}

TEST(SensorModelCoreTest, NoiseAppliedConsistentlyInBothFrames) {
    // 车在原位、theta=0 ⇒ x_base 的偏移应与 global 的偏移一致（同一 n_x/n_y）
    const VehicleState car{.x = 0.0, .y = 0.0, .theta = 0.0, .v = 0.0};
    const std::vector<TrackCone> cones = {TrackCone{.x = 5.0, .y = 3.0, .type = 0, .id = 1}};
    std::mt19937 rng{42};

    const auto detected = PredictVisibleCones(cones, car, 120.0, 15.0, 0.2, rng);

    ASSERT_EQ(detected.size(), 1u);
    const float n_x = detected[0].x_base - 5.0f;
    const float n_y = detected[0].y_base - 3.0f;
    EXPECT_NEAR(detected[0].x_global - 5.0f, n_x, 1e-5);
    EXPECT_NEAR(detected[0].y_global - 3.0f, n_y, 1e-5);
    // 噪声确实被注入（非零）
    EXPECT_GT(std::abs(n_x) + std::abs(n_y), 0.0f);
}
