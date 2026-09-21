#include "mpc_controller/qp_solver.hpp"

#include <limits>

namespace mpc {

QpResult BoxQpSolver::Solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub, const std::optional<Eigen::VectorXd>& warm_x) {
    const auto m = H.rows();
    QpResult result;
    result.x = Eigen::VectorXd::Zero(m);

    if (m == 0 || H.cols() != m || g.size() != m || lb.size() != m || ub.size() != m) {
        result.converged = false;
        return result;
    }

    // 1. 构造增广系统矩阵 H_rho = H + rho * I
    const double rho = (settings_.rho > 1e-6) ? settings_.rho : 1.0;
    Eigen::MatrixXd H_rho = H + rho * Eigen::MatrixXd::Identity(m, m);

    // 2. Cholesky 分解 (LLT)
    Eigen::LLT<Eigen::MatrixXd> llt(H_rho);
    if (llt.info() != Eigen::Success) {
        // 如果非严格正定，添加对角微小正则化保证数值鲁棒性
        H_rho += 1e-4 * Eigen::MatrixXd::Identity(m, m);
        llt.compute(H_rho);
        if (llt.info() != Eigen::Success) {
            result.converged = false;
            return result;
        }
    }

    // 3. 状态变量与热启动初始化
    Eigen::VectorXd u = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(m);
    if (warm_x && warm_x->size() == m) {
        z = warm_x->cwiseMax(lb).cwiseMin(ub);
    } else if (z_.size() == m) {
        z = z_.cwiseMax(lb).cwiseMin(ub);
    } else {
        z = (0.5 * (lb + ub)).eval();
    }

    Eigen::VectorXd y = (y_.size() == m) ? y_ : Eigen::VectorXd::Zero(m);

    // 4. ADMM 迭代循环
    bool converged = false;
    size_t iter = 0;
    double final_primal_residual = std::numeric_limits<double>::infinity();
    double final_dual_residual = std::numeric_limits<double>::infinity();
    double final_eps_pri = std::numeric_limits<double>::infinity();
    double final_eps_dual = std::numeric_limits<double>::infinity();
    for (; iter < settings_.max_iter; ++iter) {
        // Step 1: u 子问题求解 (通过已分解的 Cholesky 因子做极速三角前向/后向代入)
        Eigen::VectorXd rhs = rho * z - y - g;
        u = llt.solve(rhs);

        // Step 2: z 子问题投影截断 (闭式投影解析解)
        Eigen::VectorXd z_prev = z;
        z = (u + y / rho).cwiseMax(lb).cwiseMin(ub);

        // Step 3: y 对偶乘子更新
        Eigen::VectorXd r_pri_vec = u - z;
        y += rho * r_pri_vec;

        // Step 4: 残差与停机准则判定
        double r_pri = r_pri_vec.template lpNorm<Eigen::Infinity>();
        double r_dual = (rho * (z - z_prev)).template lpNorm<Eigen::Infinity>();

        double eps_pri = settings_.eps_abs + settings_.eps_rel * std::max(u.template lpNorm<Eigen::Infinity>(),
                                                                          z.template lpNorm<Eigen::Infinity>());
        double eps_dual = settings_.eps_abs + settings_.eps_rel * y.template lpNorm<Eigen::Infinity>();
        final_primal_residual = r_pri;
        final_dual_residual = r_dual;
        final_eps_pri = eps_pri;
        final_eps_dual = eps_dual;

        if (r_pri <= eps_pri && r_dual <= eps_dual) {
            converged = true;
            iter++;
            break;
        }
    }

    // 5. 保存对偶状态用于下个控制周期的 Warm-Start
    z_ = z;
    y_ = y;

    result.x = z;
    result.iterations = iter;
    result.converged = converged;
    result.cost = 0.5 * z.dot(H * z) + g.dot(z);
    result.primal_residual = final_primal_residual;
    result.dual_residual = final_dual_residual;
    result.primal_tolerance = final_eps_pri;
    result.dual_tolerance = final_eps_dual;

    return result;
}

}  // namespace mpc
