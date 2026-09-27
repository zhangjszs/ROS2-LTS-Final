#include <gtest/gtest.h>

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
