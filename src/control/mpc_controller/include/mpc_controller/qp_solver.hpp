#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace mpc {

/**
 * @brief QP 求解设置选项
 */
struct QpSettings {
    double rho{1.0};        // ADMM 惩罚参数
    double eps_abs{1e-4};   // 绝对收敛容差
    double eps_rel{1e-4};   // 相对收敛容差
    size_t max_iter{50};    // 最大迭代步数
    bool warm_start{true};  // 是否启用热启动
    double acceptable_primal_residual{0.25};
    double acceptable_dual_residual{0.01};
};

/**
 * @brief QP 求解结果状态
 */
struct QpResult {
    Eigen::VectorXd x;      // 最优决策变量
    size_t iterations{0};   // 实际迭代次数
    bool converged{false};  // 是否满足收敛准则
    double cost{0.0};       // 最终代价值
    double primal_residual{0.0};
    double dual_residual{0.0};
    double primal_tolerance{0.0};
    double dual_tolerance{0.0};
};

/**
 * @brief 原生 C++20 高性能 ADMM 盒约束二次规划求解器
 * 求解标准形: min 1/2 * u^T * H * u + g^T * u,  s.t.  lb <= u <= ub
 */
class BoxQpSolver {
   public:
    explicit BoxQpSolver(QpSettings settings = QpSettings{}) : settings_(std::move(settings)) {}

    /**
     * @brief 求解盒约束凸二次规划
     * @param H 正定对称 Hessian 矩阵 (m x m)
     * @param g 线性项向量 (m)
     * @param lb 下界向量 (m)
     * @param ub 上界向量 (m)
     * @param warm_x 可选的热启动初值向量
     */
    [[nodiscard]] QpResult Solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::VectorXd& lb,
                                 const Eigen::VectorXd& ub,
                                 const std::optional<Eigen::VectorXd>& warm_x = std::nullopt);

    void SetSettings(const QpSettings& settings) noexcept { settings_ = settings; }
    [[nodiscard]] const QpSettings& GetSettings() const noexcept { return settings_; }

   private:
    QpSettings settings_;
    Eigen::VectorXd y_;  // 对偶变量 (Lagrange 乘子)
    Eigen::VectorXd z_;  // 分裂松弛变量
};

}  // namespace mpc
