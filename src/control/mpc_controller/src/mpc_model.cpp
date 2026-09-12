#include "mpc_controller/mpc_model.hpp"

#include <chrono>

namespace mpc {

MpcSolution MpcModel::Step(double current_x, double current_y, double current_theta, double current_speed,
                           double prev_steer, double prev_accel, const std::vector<ReferencePoint>& reference_path) {
    auto start_time = std::chrono::steady_clock::now();
    MpcSolution solution;

    if (reference_path.size() < 2) {
        solution.success = false;
        return solution;
    }

    const size_t Np = config_.horizon.Np;
    const size_t Nc = config_.horizon.Nc;
    const double Ts = config_.horizon.Ts;
    const double L = (config_.system.wheelbase > 0.5) ? config_.system.wheelbase : 1.53;

    // 1. 寻找参考路径上距离车辆当前位置最近的航路点
    size_t best_idx = 0;
    double min_dist_sq = 1e18;
    for (size_t i = 0; i < reference_path.size(); ++i) {
        double dx = current_x - reference_path[i].x;
        double dy = current_y - reference_path[i].y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_dist_sq) {
            min_dist_sq = d2;
            best_idx = i;
        }
    }

    // 2. 截取或沿切线外推未来 Np 个参考航路点
    std::vector<ReferencePoint> ref_horizon;
    ref_horizon.reserve(Np);
    const double target_v = (config_.horizon.target_speed > 0.5) ? config_.horizon.target_speed : 8.0;

    for (size_t k = 0; k < Np; ++k) {
        size_t idx = best_idx + k;
        if (idx < reference_path.size()) {
            ReferencePoint pt = reference_path[idx];
            if (pt.speed <= 0.1) {
                pt.speed = target_v;
            }
            ref_horizon.push_back(pt);
        } else {
            // 路径末端按最后已知航向与步长线性外推
            const auto& last = ref_horizon.empty() ? reference_path.back() : ref_horizon.back();
            double ds = target_v * Ts;
            ReferencePoint ext;
            ext.x = last.x + ds * std::cos(last.theta);
            ext.y = last.y + ds * std::sin(last.theta);
            ext.theta = last.theta;
            ext.curvature = 0.0;
            ext.speed = target_v;
            ref_horizon.push_back(ext);
        }
    }
    solution.reference_horizon = ref_horizon;

    // 3. 计算当前时刻初始横向误差与航向偏差 x0 = [e_y, e_psi, e_v]^T
    const auto& ref0 = ref_horizon.front();
    double dx = current_x - ref0.x;
    double dy = current_y - ref0.y;
    double e_y = -dx * std::sin(ref0.theta) + dy * std::cos(ref0.theta);
    double e_psi = NormalizeAngle(current_theta - ref0.theta);
    double e_v = current_speed - ref0.speed;

    Eigen::VectorXd x0(kNx);
    x0 << e_y, e_psi, e_v;

    // 4. 离散线性时变 (LTV) 误差动力学方程构造
    // x_{k+1} = A_k * x_k + B_k * u_k + d_k
    std::vector<Eigen::MatrixXd> A_list(Np);
    std::vector<Eigen::MatrixXd> B_list(Np);
    std::vector<Eigen::VectorXd> d_list(Np);

    double v_eff = std::max(std::abs(current_speed), 0.5);

    for (size_t k = 0; k < Np; ++k) {
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(kNx, kNx);
        A(0, 1) = v_eff * Ts;
        A_list[k] = A;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(kNx, kNu);
        B(1, 0) = (v_eff * Ts) / L;
        B(2, 1) = Ts;
        B_list[k] = B;

        Eigen::VectorXd d = Eigen::VectorXd::Zero(kNx);
        d(1) = -v_eff * ref_horizon[k].curvature * Ts;
        d_list[k] = d;
    }

    // 5. 凝聚形式预测方程构造: X = Phi * x0 + Gamma * U + D
    const int total_x = static_cast<int>(kNx * Np);
    const int total_u = static_cast<int>(kNu * Nc);

    Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(total_x, kNx);
    Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(total_x, total_u);
    Eigen::VectorXd D = Eigen::VectorXd::Zero(total_x);

    Eigen::MatrixXd A_cum = Eigen::MatrixXd::Identity(kNx, kNx);
    Eigen::VectorXd d_cum = Eigen::VectorXd::Zero(kNx);

    for (size_t i = 0; i < Np; ++i) {
        d_cum = A_list[i] * d_cum + d_list[i];
        A_cum = A_list[i] * A_cum;

        Phi.block(i * kNx, 0, kNx, kNx) = A_cum;
        D.segment(i * kNx, kNx) = d_cum;

        for (size_t j = 0; j < Nc; ++j) {
            if (i < j)
                continue;

            Eigen::MatrixXd A_prod = Eigen::MatrixXd::Identity(kNx, kNx);
            for (size_t k = j + 1; k <= i; ++k) {
                A_prod = A_list[k] * A_prod;
            }

            if (j == Nc - 1) {
                // 控制时域到达后控制量保持常数 u_k = u_{Nc-1}
                Gamma.block(i * kNx, j * kNu, kNx, kNu) += A_prod * B_list[j];
            } else {
                Gamma.block(i * kNx, j * kNu, kNx, kNu) = A_prod * B_list[j];
            }
        }
    }

    // 6. 构造 QP 目标函数权重矩阵
    Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(total_x, total_x);
    for (size_t i = 0; i < Np; ++i) {
        Q_bar(i * kNx + 0, i * kNx + 0) = config_.weights.q_lat;
        Q_bar(i * kNx + 1, i * kNx + 1) = config_.weights.q_heading;
        Q_bar(i * kNx + 2, i * kNx + 2) = config_.weights.q_speed;
    }

    Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(total_u, total_u);
    for (size_t j = 0; j < Nc; ++j) {
        R_bar(j * kNu + 0, j * kNu + 0) = config_.weights.r_steer;
        R_bar(j * kNu + 1, j * kNu + 1) = config_.weights.r_accel;
    }

    // 差分矩阵 S: Delta U = S * U - S0 * u_prev
    Eigen::MatrixXd S = Eigen::MatrixXd::Identity(total_u, total_u);
    for (size_t j = 1; j < Nc; ++j) {
        S.block(j * kNu, (j - 1) * kNu, kNu, kNu) = -Eigen::MatrixXd::Identity(kNu, kNu);
    }

    Eigen::MatrixXd S0 = Eigen::MatrixXd::Zero(total_u, kNu);
    S0.block(0, 0, kNu, kNu) = Eigen::MatrixXd::Identity(kNu, kNu);

    Eigen::MatrixXd Rd_bar = Eigen::MatrixXd::Zero(total_u, total_u);
    for (size_t j = 0; j < Nc; ++j) {
        Rd_bar(j * kNu + 0, j * kNu + 0) = config_.weights.rd_steer;
        Rd_bar(j * kNu + 1, j * kNu + 1) = config_.weights.rd_accel;
    }

    Eigen::VectorXd u_prev_vec(kNu);
    u_prev_vec << prev_steer, prev_accel;

    // Hessian: H = 2 * (Gamma^T * Q_bar * Gamma + R_bar + S^T * Rd_bar * S)
    Eigen::MatrixXd H = 2.0 * (Gamma.transpose() * Q_bar * Gamma + R_bar + S.transpose() * Rd_bar * S);

    // Gradient: g = 2 * Gamma^T * Q_bar * (Phi * x0 + D) - 2 * S^T * Rd_bar * S0 * u_prev
    Eigen::VectorXd g =
        2.0 * Gamma.transpose() * Q_bar * (Phi * x0 + D) - 2.0 * S.transpose() * Rd_bar * S0 * u_prev_vec;

    // 7. 物理约束与转向速率边界构造
    Eigen::VectorXd lb = Eigen::VectorXd::Zero(total_u);
    Eigen::VectorXd ub = Eigen::VectorXd::Zero(total_u);

    const double steer_max = config_.limits.max_steer_rad;
    const double d_steer_step = config_.limits.max_steer_rate * Ts;
    const double accel_min = config_.limits.min_accel;
    const double accel_max = config_.limits.max_accel;

    double cur_steer_ref = prev_steer;

    for (size_t j = 0; j < Nc; ++j) {
        double steer_lb = std::max(-steer_max, cur_steer_ref - d_steer_step * static_cast<double>(j + 1));
        double steer_ub = std::min(steer_max, cur_steer_ref + d_steer_step * static_cast<double>(j + 1));

        lb(j * kNu + 0) = steer_lb;
        ub(j * kNu + 0) = steer_ub;

        lb(j * kNu + 1) = accel_min;
        ub(j * kNu + 1) = accel_max;
    }

    // 8. 调用 ADMM QP 求解器
    QpResult qp_res = qp_solver_.Solve(H, g, lb, ub);

    if (!qp_res.converged && qp_res.iterations >= qp_solver_.GetSettings().max_iter) {
        // 如果未严格收敛但得到了可行解，进行平滑保护
        solution.steering_rad = std::clamp(qp_res.x(0), lb(0), ub(0));
        solution.accel_mps2 = std::clamp(qp_res.x(1), lb(1), ub(1));
        solution.success = true;
    } else {
        solution.steering_rad = qp_res.x(0);
        solution.accel_mps2 = qp_res.x(1);
        solution.success = qp_res.converged;
    }

    // 9. 基于解出的最优前瞻控制序列进行运动学时域轨迹前向积分推演
    std::vector<PredictedPoint> pred_path;
    pred_path.reserve(Np);

    double px = current_x;
    double py = current_y;
    double ptheta = current_theta;
    double pv = current_speed;

    for (size_t k = 0; k < Np; ++k) {
        size_t u_idx = std::min(k, Nc - 1);
        double delta_k = qp_res.x(u_idx * kNu + 0);
        double a_k = qp_res.x(u_idx * kNu + 1);

        px += pv * std::cos(ptheta) * Ts;
        py += pv * std::sin(ptheta) * Ts;
        ptheta = NormalizeAngle(ptheta + (pv / L) * std::tan(delta_k) * Ts);
        pv = std::max(0.0, pv + a_k * Ts);

        pred_path.push_back({.x = px, .y = py, .theta = ptheta, .speed = pv});
    }

    solution.predicted_trajectory = std::move(pred_path);

    auto end_time = std::chrono::steady_clock::now();
    solution.solve_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    return solution;
}

}  // namespace mpc
