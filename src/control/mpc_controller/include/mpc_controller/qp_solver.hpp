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
 * @brief 一次求解被接受的方式（#47：`success` 必须能区分"真收敛"与"落在兜底带内"）
 *
 * 背景：`eps_abs/eps_rel` 是目标收敛容差，而 `acceptable_*` 是"用尽迭代也不放弃指令"
 * 的兜底带（绝对量、与问题尺度无关）。两者相差几个量级时，只看 `converged/success`
 * 会把"兜底接受的欠收敛解"和"真收敛解"混成一个指标（#44 的"未收敛率"因此被误读过）。
 */
enum class QpAcceptance {
    // ADMM 未达收敛且残差超出兜底带：调用方必须按"没有新指令"处理。
    kRejected = 0,
    // 达到 eps_abs/eps_rel 停机准则。
    kConverged = 1,
    // 用尽 max_iter 未收敛，但残差在 acceptable_* 兜底带内：被接受，但解质量不保证。
    kAcceptedApproximation = 2,
};

/**
 * @brief 按兜底判据给一次 QP 结果定级（唯一判定点，`MpcModel::Step` 也走它）
 *
 * 单调性契约：同一实例上 `max_iter` 增大只会让定级从 kRejected 往 kConverged 方向走
 * （更多迭代 ⇒ ADMM 终端残差单调不增），不会反过来；由单测锁住（#47）。
 */
[[nodiscard]] inline QpAcceptance ClassifyQpAcceptance(const QpResult& result, const QpSettings& settings) noexcept {
    if (result.converged) {
        return QpAcceptance::kConverged;
    }
    const bool exhausted = result.iterations >= settings.max_iter;
    const bool within_band = exhausted && result.primal_residual <= settings.acceptable_primal_residual &&
                             result.dual_residual <= settings.acceptable_dual_residual;
    return within_band ? QpAcceptance::kAcceptedApproximation : QpAcceptance::kRejected;
}

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

    /**
     * @brief 清空跨问题残留的 ADMM 内状态 (z/y)
     * 盒约束 ADMM 的热启动只对"同一问题的连续求解"有效；若用同一实例求解另一个
     * QP（如纠偏符号翻转），维数相同但携带的对偶状态会污染迭代。
     */
    void Reset() noexcept {
        y_ = Eigen::VectorXd();
        z_ = Eigen::VectorXd();
    }

   private:
    QpSettings settings_;
    Eigen::VectorXd y_;  // 对偶变量 (Lagrange 乘子)
    Eigen::VectorXd z_;  // 分裂松弛变量
};

}  // namespace mpc
