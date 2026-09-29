#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

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

// ============================================================================
// #47：接受判据的单调性与"怎么被接受的"可观测性。
//
// 起因：#46 在调参集上观测到 max_iter 50→150 使未收敛率 19.45%→41.39%，据此怀疑
// 判据本身非单调。实测结论（见 docs/MPC_TUNING_FREEZE.md 的 #47 节）是：
// **同一批 QP 实例上判据严格单调**（0 例反例），那个 19%→41% 是跨轨迹的闭环统计量
// ——解得更准 ⇒ 车更快 ⇒ 更早冲出走廊 ⇒ 后续 QP 更难。本组用例把这条契约钉住，
// 并把"success 不等于解收敛"这件事变成机器可读（否则未收敛率会被读成收敛率）。
// ============================================================================
namespace {

// 固定（无随机）曲率参考路径：正弦复合曲率，够难、可复现。
[[nodiscard]] std::vector<ReferencePoint> CurvedPath(size_t n = 900, double ds = 0.25) {
    std::vector<ReferencePoint> path;
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    for (size_t i = 0; i < n; ++i) {
        const double t = static_cast<double>(i);
        const double k = 0.20 * std::sin(2.0 * M_PI * t / 180.0) + 0.06 * std::sin(2.0 * M_PI * t / 47.0);
        path.push_back({.x = x, .y = y, .theta = th, .curvature = k, .speed = 8.0, .speed_valid = true});
        th += k * ds;
        x += std::cos(th) * ds;
        y += std::sin(th) * ds;
    }
    return path;
}

// 固定状态序列（开环，不随求解结果漂移）⇒ 三档 max_iter 看到的是同一批 QP 实例。
struct ProbeState {
    double idx_frac;
    double lat_err;
    double heading_err;
    double speed;
    double prev_steer;
    double prev_accel;
};

[[nodiscard]] std::vector<ProbeState> HardInstanceSet() {
    std::vector<ProbeState> set;
    for (int t = 0; t < 60; ++t) {
        const double f = static_cast<double>(t);
        set.push_back({.idx_frac = std::fmod(f * 0.0137, 0.8),
                       .lat_err = 0.35 * std::sin(2.0 * M_PI * f / 9.0),
                       .heading_err = 0.08 * std::sin(2.0 * M_PI * f / 6.0),
                       .speed = 8.0 + 3.5 * std::sin(2.0 * M_PI * f / 21.0),
                       .prev_steer = 0.10 * std::sin(2.0 * M_PI * f / 8.0),
                       .prev_accel = 1.0 * std::sin(2.0 * M_PI * f / 5.0)});
    }
    return set;
}

// 用给定 max_iter 逐个解同一批实例，返回逐实例的接受位图与接受总数。
[[nodiscard]] std::pair<std::vector<int>, int> AcceptanceAt(const std::vector<ReferencePoint>& path, size_t max_iter) {
    MpcModel model(OfflineConfig());
    QpSettings qs = model.GetQpSettings();
    qs.max_iter = max_iter;
    model.SetQpSettings(qs);

    std::vector<int> accepted;
    int total = 0;
    for (const auto& s : HardInstanceSet()) {
        const size_t idx = static_cast<size_t>(s.idx_frac * static_cast<double>(path.size()));
        const ReferencePoint& rp = path[idx];
        const double cx = rp.x - s.lat_err * std::sin(rp.theta);
        const double cy = rp.y + s.lat_err * std::cos(rp.theta);
        const auto sol = model.Step(cx, cy, rp.theta + s.heading_err, s.speed, s.prev_steer, s.prev_accel, path);
        accepted.push_back(sol.success ? 1 : 0);
        total += sol.success ? 1 : 0;
    }
    return {std::move(accepted), total};
}

}  // namespace

// 实例级单调性契约：同一批 QP 上，迭代预算变大只会让"接受"变多，绝不能把
// 上一档能用的解判成不可用（#47 标题现象若不成立即由本用例守住）。
TEST(MpcAcceptanceContract, LargerIterationBudgetNeverLosesAcceptance) {
    const auto path = CurvedPath();
    const auto [ok50, n50] = AcceptanceAt(path, 50);
    const auto [ok150, n150] = AcceptanceAt(path, 150);
    const auto [ok300, n300] = AcceptanceAt(path, 300);

    ASSERT_EQ(ok50.size(), ok150.size());
    ASSERT_EQ(ok150.size(), ok300.size());
    for (size_t i = 0; i < ok50.size(); ++i) {
        EXPECT_LE(ok50[i], ok150[i]) << "实例 " << i << "：max_iter 50→150 反而拒收";
        EXPECT_LE(ok150[i], ok300[i]) << "实例 " << i << "：max_iter 150→300 反而拒收";
    }
    EXPECT_LE(n50, n150);
    EXPECT_LE(n150, n300);
    // 用例集合必须真的"难"：若三档全都接受，本用例形同空转（负样本自检）。
    EXPECT_LT(n50, static_cast<int>(ok50.size()));
}

// success 必须能区分"真收敛"与"兜底带内接受"：判据的 acceptable_* 是绝对量
// （primal 0.25 ≈ 满舵 0.40 的 62%），只看 success 会把欠收敛解当成解好了。
TEST(MpcAcceptanceContract, SuccessReportsHowItWasAccepted) {
    const auto path = CurvedPath();
    const auto straight = StraightPath(50.0, 8.0);
    const size_t idx = static_cast<size_t>(0.11 * static_cast<double>(path.size()));
    const ReferencePoint& rp = path[idx];
    const double cx = rp.x - 0.2 * std::sin(rp.theta);
    const double cy = rp.y + 0.2 * std::cos(rp.theta);

    // (a) 平凡情形（在参考线上、速度已达目标、无历史指令）：真收敛，且没花完迭代。
    MpcModel easy_model(OfflineConfig());
    const auto sol_easy = easy_model.Step(10.0, 0.0, 0.0, 8.0, 0.0, 0.0, straight);
    ASSERT_TRUE(sol_easy.success);
    EXPECT_EQ(sol_easy.qp_acceptance, QpAcceptance::kConverged);
    EXPECT_LT(sol_easy.qp_iterations, easy_model.GetQpSettings().max_iter);

    // (b) 默认参数下的"在弯道里偏 0.2 m"：仍 success，但定级必须是"兜底接受"。
    //     实测残差 0.079，而它自己的收敛目标是 ~1.06e-4——差 ≈750 倍，且残差比
    //     实际下发的转角（顶在速率边界 0.060）还大。实跑 trackdrive@0.9（k=50，725 拍）
    //     的拆分是 457 converged + 127 accepted_approx + 141 failures——只看"未收敛率
    //     19.45%"会把那 127 拍当成收敛解。
    MpcModel model(OfflineConfig());
    const auto sol_approx = model.Step(cx, cy, rp.theta, 8.0, 0.0, 0.0, path);
    ASSERT_TRUE(sol_approx.success);
    EXPECT_EQ(sol_approx.qp_acceptance, QpAcceptance::kAcceptedApproximation);
    EXPECT_EQ(sol_approx.qp_iterations, model.GetQpSettings().max_iter);
    EXPECT_LE(sol_approx.qp_primal_residual, model.GetQpSettings().acceptable_primal_residual);
    EXPECT_GT(sol_approx.qp_primal_residual, 100.0 * sol_approx.qp_primal_tolerance);
    EXPECT_GT(sol_approx.qp_primal_tolerance, 0.0);
    // 残差比本拍指令本身还大：指令完全由箱约束投影决定，不是 QP 最优解的收敛结果。
    EXPECT_GT(sol_approx.qp_primal_residual, std::abs(sol_approx.steering_rad));

    // (c) 拒收也必须可辨：6m 横向误差超出收敛半径 ⇒ kRejected，且残差仍然透出
    //     （诊断侧要能回答"差多少"，而不是只知道没成功）。
    const auto sol_reject =
        model.Step(rp.x - 6.0 * std::sin(rp.theta), rp.y + 6.0 * std::cos(rp.theta), rp.theta, 8.0, 0.0, 0.0, path);
    ASSERT_FALSE(sol_reject.success);
    EXPECT_EQ(sol_reject.qp_acceptance, QpAcceptance::kRejected);
    EXPECT_TRUE(std::isfinite(sol_reject.qp_primal_residual));
    EXPECT_TRUE(std::isfinite(sol_reject.qp_dual_residual));
    EXPECT_GT(sol_reject.qp_primal_residual, model.GetQpSettings().acceptable_primal_residual);

    // (d) 退化输入根本没解过：定级保持 kRejected，不得留下"曾收敛"的假象。
    const auto sol_degenerate = model.Step(1.0, 0.0, 0.0, 8.0, 0.0, 0.0, {});
    EXPECT_FALSE(sol_degenerate.success);
    EXPECT_EQ(sol_degenerate.qp_acceptance, QpAcceptance::kRejected);
    EXPECT_EQ(sol_degenerate.qp_iterations, 0u);
}
