#include <gtest/gtest.h>

#include <cmath>

#include "mpc_controller/qp_solver.hpp"

using namespace mpc;

TEST(BoxQpSolverTest, UnconstrainedOptimum) {
    BoxQpSolver solver;

    // 目标函数: min 1/2 * (2*x1^2 + 2*x2^2) - 4*x1 - 6*x2
    // 解析极值点: x1* = 2.0, x2* = 3.0
    Eigen::MatrixXd H(2, 2);
    H << 2.0, 0.0, 0.0, 2.0;

    Eigen::VectorXd g(2);
    g << -4.0, -6.0;

    Eigen::VectorXd lb(2);
    lb << -10.0, -10.0;

    Eigen::VectorXd ub(2);
    ub << 10.0, 10.0;

    auto res = solver.Solve(H, g, lb, ub);

    EXPECT_TRUE(res.converged);
    EXPECT_NEAR(res.x(0), 2.0, 1e-3);
    EXPECT_NEAR(res.x(1), 3.0, 1e-3);
}

TEST(BoxQpSolverTest, BoundedBoxClamping) {
    BoxQpSolver solver;

    // 解析无约束极值点在 (2.0, 3.0)，但上下界被限制在 [-1.0, 1.5]
    Eigen::MatrixXd H(2, 2);
    H << 2.0, 0.0, 0.0, 2.0;

    Eigen::VectorXd g(2);
    g << -4.0, -6.0;

    Eigen::VectorXd lb(2);
    lb << -1.0, -1.0;

    Eigen::VectorXd ub(2);
    ub << 1.5, 1.5;

    auto res = solver.Solve(H, g, lb, ub);

    EXPECT_TRUE(res.converged);
    EXPECT_NEAR(res.x(0), 1.5, 1e-3);
    EXPECT_NEAR(res.x(1), 1.5, 1e-3);
}

TEST(BoxQpSolverTest, HighDimensionalQpPerformance) {
    BoxQpSolver solver;
    const int m = 30;  // 对应 15 步控制时域

    // 构造正定对称矩阵
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(m, m);
    Eigen::MatrixXd H = A.transpose() * A + 2.0 * Eigen::MatrixXd::Identity(m, m);
    Eigen::VectorXd g = Eigen::VectorXd::Random(m);

    Eigen::VectorXd lb = -0.5 * Eigen::VectorXd::Ones(m);
    Eigen::VectorXd ub = 0.5 * Eigen::VectorXd::Ones(m);

    auto res = solver.Solve(H, g, lb, ub);

    EXPECT_TRUE(res.converged);
    for (int i = 0; i < m; ++i) {
        EXPECT_GE(res.x(i), lb(i) - 1e-4);
        EXPECT_LE(res.x(i), ub(i) + 1e-4);
    }
}

TEST(BoxQpSolverTest, ReportsNonConvergenceEvidenceAtIterationLimit) {
    QpSettings settings;
    settings.max_iter = 1;
    BoxQpSolver solver(settings);

    Eigen::MatrixXd H = 2.0 * Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd g = Eigen::VectorXd::Constant(4, -4.0);
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(4, -10.0);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(4, 10.0);

    const auto result = solver.Solve(H, g, lb, ub);

    EXPECT_FALSE(result.converged);
    EXPECT_EQ(result.iterations, 1u);
    EXPECT_TRUE(std::isfinite(result.primal_residual));
    EXPECT_TRUE(std::isfinite(result.dual_residual));
    EXPECT_GT(result.primal_tolerance, 0.0);
    EXPECT_GT(result.dual_tolerance, 0.0);
}

// #19 B1：耦合 2x2 小问题，解析最优只有一个分量贴边（可用 KKT 手验）：
// min 1/2 x^T [[4,1],[1,3]] x - [8,6]^T x, s.t. x in [-1.5, 1.5] x [-2, 2]
// x1* = 1.5（上界活动，梯度 4*1.5+1.5-8 = -0.5 ≤ 0 与上界一致），
// x2* = (6 - x1)/3 = 1.5（内部，梯度为 0）。
TEST(BoxQpSolverTest, AnalyticSolutionWithSingleActiveBound) {
    BoxQpSolver solver;

    Eigen::MatrixXd H(2, 2);
    H << 4.0, 1.0, 1.0, 3.0;
    Eigen::VectorXd g(2);
    g << -8.0, -6.0;
    Eigen::VectorXd lb(2);
    lb << -1.5, -2.0;
    Eigen::VectorXd ub(2);
    ub << 1.5, 2.0;

    const auto res = solver.Solve(H, g, lb, ub);

    ASSERT_TRUE(res.converged);
    EXPECT_NEAR(res.x(0), 1.5, 5e-3);
    EXPECT_NEAR(res.x(1), 1.5, 5e-3);
}

// #19 B1：Reset() 必须丢弃跨问题残留的内状态：同一实例解完一个 QP 后
// Reset 再解原问题，结果与首解一致（不热启动不可幂等则说明状态残留）。
TEST(BoxQpSolverTest, ResetDropsCarriedOverAdmmState) {
    BoxQpSolver solver;
    Eigen::MatrixXd H(2, 2);
    H << 2.0, 0.0, 0.0, 2.0;
    Eigen::VectorXd g(2);
    g << -4.0, -6.0;
    Eigen::VectorXd lb(2);
    lb << -10.0, -10.0;
    Eigen::VectorXd ub(2);
    ub << 10.0, 10.0;

    const auto first = solver.Solve(H, g, lb, ub);
    // 中途去解一个异号、同维的无关问题，然后 Reset 回到原问题
    Eigen::VectorXd g_neg = -g;
    solver.Solve(H, g_neg, lb, ub);
    solver.Reset();
    const auto after = solver.Solve(H, g, lb, ub);

    ASSERT_TRUE(after.converged);
    EXPECT_NEAR(after.x(0), first.x(0), 1e-9);
    EXPECT_NEAR(after.x(1), first.x(1), 1e-9);
}

// #52：warm_start=false 必须忽略并清空内部 z_/y_——同一实例上即便此前（warm_start=true）
// 留下了非零残留，也应从盒中心冷启动，等价于"每次调用前 Reset()"。用 max_iter=1 放大初始
// 状态对结果的影响，使"是否复用残留"可被逐位区分。
TEST(BoxQpSolverTest, WarmStartDisabledIgnoresAndClearsCarriedState) {
    const Eigen::MatrixXd H = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd g(2);
    g << -4.0, -6.0;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(2, -10.0);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(2, 10.0);

    QpSettings warm_on;
    warm_on.max_iter = 1;  // warm_start 默认 true
    BoxQpSolver solver(warm_on);
    Eigen::VectorXd g_seed(2);
    g_seed << 7.0, -3.0;
    [[maybe_unused]] const auto seeded = solver.Solve(H, g_seed, lb, ub);  // 制造并留存内部 z_/y_ 残留

    QpSettings warm_off = warm_on;
    warm_off.warm_start = false;
    solver.SetSettings(warm_off);
    const auto off = solver.Solve(H, g, lb, ub);

    // 参照：全新实例、同样 warm_start=false（等价于每次调用前 Reset 的冷启动）
    BoxQpSolver fresh(warm_off);
    const auto ref = fresh.Solve(H, g, lb, ub);

    EXPECT_EQ(off.iterations, ref.iterations);
    EXPECT_NEAR(off.x(0), ref.x(0), 1e-12);
    EXPECT_NEAR(off.x(1), ref.x(1), 1e-12);

    // 反证非空：保持 warm_start=true 时复用残留会得到明显不同的结果
    BoxQpSolver reuse(warm_on);
    [[maybe_unused]] const auto reused_seed = reuse.Solve(H, g_seed, lb, ub);
    const auto on = reuse.Solve(H, g, lb, ub);
    EXPECT_GT(std::abs(on.x(0) - ref.x(0)) + std::abs(on.x(1) - ref.x(1)), 1e-9);
}

// #52：显式 warm_x 是"调用方当次初值"，不随 warm_start 被关掉而失效。
TEST(BoxQpSolverTest, WarmStartDisabledStillHonorsExplicitWarmX) {
    const Eigen::MatrixXd H = 2.0 * Eigen::MatrixXd::Identity(2, 2);
    Eigen::VectorXd g(2);
    g << -4.0, -6.0;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(2, -10.0);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(2, 10.0);

    QpSettings settings;
    settings.warm_start = false;
    settings.max_iter = 1;  // 放大初值对结果的影响
    BoxQpSolver solver(settings);

    const auto cold = solver.Solve(H, g, lb, ub);

    Eigen::VectorXd warm_x(2);
    warm_x << 5.0, -5.0;  // 与盒中心 (0,0) 明显不同
    const auto warm = solver.Solve(H, g, lb, ub, warm_x);

    EXPECT_GT(std::abs(warm.x(0) - cold.x(0)) + std::abs(warm.x(1) - cold.x(1)), 1e-9);
}

// #47：接受定级是唯一判定点的纯函数形式——把"什么算被接受"钉成表，
// 避免判据在 MpcModel 与调用方两处各写一份而漂移（历史上兜底带只写在 Step 里）。
namespace {

[[nodiscard]] QpResult MakeResult(bool converged, size_t iterations, double primal, double dual) {
    QpResult r;
    r.converged = converged;
    r.iterations = iterations;
    r.primal_residual = primal;
    r.dual_residual = dual;
    return r;
}

}  // namespace

TEST(QpAcceptanceClassification, ConvergedResultIsAlwaysKConverged) {
    const QpSettings settings;  // max_iter=50, acceptable=0.25/0.01
    EXPECT_EQ(ClassifyQpAcceptance(MakeResult(true, 7, 1e-9, 1e-9), settings), QpAcceptance::kConverged);
    // 残差再大也已经是收敛事实（判据不改语义），定级仍按 converged 走
    EXPECT_EQ(ClassifyQpAcceptance(MakeResult(true, 50, 0.9, 0.9), settings), QpAcceptance::kConverged);
}

TEST(QpAcceptanceClassification, ExhaustedIterationsWithinBandIsAcceptedApproximation) {
    const QpSettings settings;
    const auto in_band = MakeResult(false, settings.max_iter, 0.2, 0.005);
    EXPECT_EQ(ClassifyQpAcceptance(in_band, settings), QpAcceptance::kAcceptedApproximation);
    // 边界值本身算"在带内"（与既有 Step 判据的 <= 一致）
    const auto on_band =
        MakeResult(false, settings.max_iter, settings.acceptable_primal_residual, settings.acceptable_dual_residual);
    EXPECT_EQ(ClassifyQpAcceptance(on_band, settings), QpAcceptance::kAcceptedApproximation);
}

TEST(QpAcceptanceClassification, OutOfBandOrPrematureStopIsRejected) {
    const QpSettings settings;
    EXPECT_EQ(ClassifyQpAcceptance(MakeResult(false, settings.max_iter, 0.3, 0.005), settings),
              QpAcceptance::kRejected);
    EXPECT_EQ(ClassifyQpAcceptance(MakeResult(false, settings.max_iter, 0.2, 0.02), settings), QpAcceptance::kRejected);
    // 没用尽迭代预算（例如 Cholesky 失败提前返回 iterations=0）不得享受兜底带
    EXPECT_EQ(ClassifyQpAcceptance(MakeResult(false, 0, 1e-12, 1e-12), settings), QpAcceptance::kRejected);
}
