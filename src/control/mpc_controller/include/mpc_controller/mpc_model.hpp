#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <vector>

#include "mpc_controller/mpc_params.hpp"
#include "mpc_controller/mpc_types.hpp"  // #22：ReferencePoint / NormalizeAngle 收敛于此（纯 std，无 Eigen）
#include "mpc_controller/qp_solver.hpp"

namespace mpc {

/**
 * @brief 预测出的未来轨迹点
 */
struct PredictedPoint {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double speed{0.0};
};

/**
 * @brief MPC 求解计算结果
 */
struct MpcSolution {
    double steering_rad{0.0};                          // 当前最优前轮转角 (rad)
    double accel_mps2{0.0};                            // 当前最优加速度 (m/s^2)
    std::vector<PredictedPoint> predicted_trajectory;  // 预测未来轨迹
    std::vector<ReferencePoint> reference_horizon;     // 参考时域轨迹
    double solve_time_ms{0.0};                         // 解算耗时 (ms)
    bool success{false};                               // 是否成功解出有效控制量
};

/**
 * @brief 运动学自行车 MPC 轨迹跟踪模型构建器
 */
class MpcModel {
   public:
    explicit MpcModel(MpcConfig config = MpcConfig{}) : config_(config), qp_solver_() {}

    void SetConfig(const MpcConfig& config) { config_ = config; }
    [[nodiscard]] const MpcConfig& GetConfig() const noexcept { return config_; }

    /**
     * @brief 执行单步 MPC 优化求解
     * @param current_x 当前自车全局 X (m)
     * @param current_y 当前自车全局 Y (m)
     * @param current_theta 当前自车全局航向角 (rad)
     * @param current_speed 当前自车速度 (m/s)
     * @param prev_steer 上一步执行的前轮转角 (rad)
     * @param prev_accel 上一步执行的加速度 (m/s^2)
     * @param reference_path 规划器输入的参考路径航路点
     */
    [[nodiscard]] MpcSolution Step(double current_x, double current_y, double current_theta, double current_speed,
                                   double prev_steer, double prev_accel,
                                   const std::vector<ReferencePoint>& reference_path);

    /**
     * @brief 角度正规化到 [-pi, pi]（唯一实现已收敛于 mpc_types.hpp；此处保留兼容转发）
     */
    [[nodiscard]] static inline double NormalizeAngle(double angle) noexcept { return mpc::NormalizeAngle(angle); }

   private:
    MpcConfig config_;
    BoxQpSolver qp_solver_;

    // 状态与输入维度定义
    static constexpr int kNx = 3;  // e_y, e_psi, e_v
    static constexpr int kNu = 2;  // delta, a
};

}  // namespace mpc
