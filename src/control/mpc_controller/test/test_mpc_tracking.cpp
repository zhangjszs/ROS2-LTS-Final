#include <gtest/gtest.h>

#include "mpc_controller/mpc_model.hpp"

using namespace mpc;

namespace {
// #19 B1 默认 fixture：与 B0 后的收敛真值/节拍一致（Ts=0.02、Np=38、Nc=25）。
[[nodiscard]] MpcConfig OfflineConfig() {
    MpcConfig config;
    config.system.wheelbase = 1.55;
    config.horizon.Np = 38;
    config.horizon.Nc = 25;
    config.horizon.Ts = 0.02;
    config.horizon.target_speed = 8.0;
    return config;
}

// 沿 X 轴的直线参考路径（逐点显式速度）。
[[nodiscard]] std::vector<ReferencePoint> StraightPath(double length_m, double speed) {
    std::vector<ReferencePoint> path;
    for (double x = 0.0; x <= length_m; x += 0.5) {
        path.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .speed = speed, .speed_valid = true});
    }
    return path;
}
}  // namespace

TEST(MpcModelTest, StraightLineLateralErrorCorrection) {
    MpcModel model(OfflineConfig());

    // 沿 X 轴的直线参考路径
    std::vector<ReferencePoint> path = StraightPath(50.0, 8.0);

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
    MpcModel model(OfflineConfig());

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

// ==================== #19 B1：数学核离线化后的行为契约 ====================
// 以下五个用例把"runner 直接调 MpcModel"所依赖的性质固化为回归门，
// 并锁定 D10（跨问题 ADMM 状态污染）修复。

// D10 回归：同一实例先解左偏再解右偏（两个独立 QP、同维数），不得因
// 上一问题的对偶状态残留而不收敛，且解应关于路径对称。
TEST(MpcCoreContractTest, ConsecutiveDistinctQpsStayConvergedAndSymmetric) {
    MpcModel model(OfflineConfig());
    const auto path = StraightPath(50.0, 8.0);

    const auto sol_left = model.Step(5.0, 0.4, 0.0, 8.0, 0.0, 0.0, path);
    const auto sol_right = model.Step(5.0, -0.4, 0.0, 8.0, 0.0, 0.0, path);
    const auto sol_left_again = model.Step(5.0, 0.4, 0.0, 8.0, 0.0, 0.0, path);

    ASSERT_TRUE(sol_left.success);
    ASSERT_TRUE(sol_right.success);
    ASSERT_TRUE(sol_left_again.success);
    EXPECT_NEAR(sol_right.steering_rad, -sol_left.steering_rad, 1e-3);      // 几何对称 → 解反对称
    EXPECT_NEAR(sol_left_again.steering_rad, sol_left.steering_rad, 1e-6);  // 与求解顺序无关
}

// 执行器约束：收敛解的输出落在转角/加速度箱式约束内（含饱和情形：
// 超速 25m/s 时减速度顶到 min_accel 箱式下界；转向受首步速率界 ±0.06）。
// 实测本求解器配置（ADMM 50 步）在横向误差 ≥0.5m 即不收敛，故饱和演示取
// 可收敛的 y=0.4，而 6m 级极端误差走可识别失败路径（见下）。
TEST(MpcCoreContractTest, ActuatorLimitsHoldUnderExtremeCrossTrackError) {
    MpcModel model(OfflineConfig());
    const auto path = StraightPath(50.0, 8.0);

    const auto sol = model.Step(5.0, 0.4, 0.0, 25.0, 0.0, 0.0, path);  // 小偏差 + 超速 → 减速饱和

    ASSERT_TRUE(sol.success);
    EXPECT_LE(std::abs(sol.steering_rad), model.GetConfig().limits.max_steer_rad + 1e-9);
    EXPECT_GE(sol.accel_mps2, model.GetConfig().limits.min_accel - 1e-9);
    EXPECT_LE(sol.accel_mps2, model.GetConfig().limits.max_accel + 1e-9);
    EXPECT_NEAR(sol.accel_mps2, model.GetConfig().limits.min_accel, 1e-9);  // 顶到制动箱式下界
}

// 不可行情景可识别：6m 级横向误差超出本求解器收敛半径时，不得返回一个
// “看起来可用”的控制量，而须 success=false + 零指令 + 空预测轨迹
// （与 DegenerateReference 同一缺陷类：不可行情形必须显式可识，runner 侧 ZOH 保持旧指令）。
TEST(MpcCoreContractTest, InfeasibleCrossTrackErrorYieldsIdentifiableFailure) {
    MpcModel model(OfflineConfig());
    const auto path = StraightPath(50.0, 8.0);

    const auto sol = model.Step(5.0, 6.0, 0.0, 8.0, 0.0, 0.0, path);

    EXPECT_FALSE(sol.success);
    EXPECT_DOUBLE_EQ(sol.steering_rad, 0.0);
    EXPECT_DOUBLE_EQ(sol.accel_mps2, 0.0);
    EXPECT_TRUE(sol.predicted_trajectory.empty());
}

// 时域末端：参考路径短于预测时域时，沿最后航向等步长外推，仍产出完整 Np 长的
// 参考/预测时域（runner 在赛道末端与短路径场景下依赖此行为不崩）。
TEST(MpcCoreContractTest, HorizonEndsAreExtrapolatedNotTruncated) {
    MpcModel model(OfflineConfig());
    const size_t Np = model.GetConfig().horizon.Np;
    const std::vector<ReferencePoint> short_path{{.x = 5.0, .y = 0.0, .theta = 0.0, .speed = 8.0, .speed_valid = true},
                                                 {.x = 6.0, .y = 0.0, .theta = 0.0, .speed = 8.0, .speed_valid = true}};

    const auto sol = model.Step(5.0, 0.0, 0.0, 8.0, 0.0, 0.0, short_path);

    ASSERT_TRUE(sol.success);
    ASSERT_EQ(sol.reference_horizon.size(), Np);
    ASSERT_EQ(sol.predicted_trajectory.size(), Np);
    // 外推点沿末端航向（此处 0）单向前进，横/艏不漂移，且全部有限
    for (size_t k = 2; k < sol.reference_horizon.size(); ++k) {
        EXPECT_GT(sol.reference_horizon[k].x, sol.reference_horizon[k - 1].x);
        EXPECT_NEAR(sol.reference_horizon[k].y, 0.0, 1e-9);
        EXPECT_NEAR(sol.reference_horizon[k].theta, 0.0, 1e-9);
    }
}

// 求解失败路径可构造：退化输入（不足 2 点）必须给出 success=false 且零指令，
// 而不是"返一个看起来可用的控制量"（#11/#24 同一缺陷类：不可行情形必须显式可识）。
TEST(MpcCoreContractTest, DegenerateReferenceYieldsIdentifiableFailure) {
    MpcModel model(OfflineConfig());
    const std::vector<ReferencePoint> one_point{{.x = 1.0, .y = 0.0, .theta = 0.0, .speed = 8.0, .speed_valid = true}};

    const auto sol_empty = model.Step(1.0, 0.0, 0.0, 8.0, 0.0, 0.0, {});
    const auto sol_single = model.Step(1.0, 0.0, 0.0, 8.0, 0.0, 0.0, one_point);

    EXPECT_FALSE(sol_empty.success);
    EXPECT_FALSE(sol_single.success);
    EXPECT_DOUBLE_EQ(sol_single.steering_rad, 0.0);
    EXPECT_DOUBLE_EQ(sol_single.accel_mps2, 0.0);
    EXPECT_TRUE(sol_single.predicted_trajectory.empty());
}
