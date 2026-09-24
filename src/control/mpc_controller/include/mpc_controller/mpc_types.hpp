#pragma once
// ============================================================================
// issue #22：MPC 核心数值类型与角度原语（纯 std，无 ROS/Eigen 依赖）。
// 从 mpc_model.hpp 迁出，使 core 模块（如 path_reference_builder.h）及其单测
// 可在不引入 MPC 求解栈（Eigen/Dense、QP 求解器）的前提下独立编译。
// mpc_model.hpp 继续提供 MpcModel::NormalizeAngle 兼容转发，行为位级不变。
// ============================================================================
#include <numbers>

namespace mpc {

/**
 * @brief 轨迹参考航路点
 */
struct ReferencePoint {
    double x{0.0};
    double y{0.0};
    double theta{0.0};
    double curvature{0.0};
    double speed{0.0};
    bool speed_valid{true};
};

/**
 * @brief 角度正规化到 [-pi, pi]
 */
[[nodiscard]] inline double NormalizeAngle(double angle) noexcept {
    constexpr double kPi = std::numbers::pi_v<double>;
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

}  // namespace mpc
