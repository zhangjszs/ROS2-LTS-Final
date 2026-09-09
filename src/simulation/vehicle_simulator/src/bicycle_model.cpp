#include "vehicle_simulator/bicycle_model.hpp"

#include <numbers>

namespace simulation {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;

/**
 * @brief 将角度归一化到 [-pi, pi]
 */
[[nodiscard]] inline double NormalizeAngle(double angle) noexcept {
    while (angle > kPi)
        angle -= 2.0 * kPi;
    while (angle < -kPi)
        angle += 2.0 * kPi;
    return angle;
}

/**
 * @brief 辅助状态累加操作 (state + dt * deriv)
 */
[[nodiscard]] inline VehicleState AccumulateState(const VehicleState& s, const VehicleState& deriv,
                                                  double dt) noexcept {
    VehicleState res;
    res.x = s.x + deriv.x * dt;
    res.y = s.y + deriv.y * dt;
    res.theta = NormalizeAngle(s.theta + deriv.theta * dt);
    res.v = std::max(0.0, s.v + deriv.v * dt);
    res.steering_angle = s.steering_angle + deriv.steering_angle * dt;
    res.accel = s.accel + deriv.accel * dt;
    res.yaw_rate = deriv.theta;
    return res;
}

}  // namespace

VehicleState BicycleModel::ComputeDerivative(const VehicleState& state, const ControlCommand& cmd) const noexcept {
    VehicleState deriv{};

    // 1. 转向执行机构一阶滞后与速率限幅
    const double target_steer_clamped =
        std::clamp(cmd.target_steering, -params_.max_steer_angle, params_.max_steer_angle);
    double raw_steer_rate = (target_steer_clamped - state.steering_angle) / params_.steer_time_const;
    deriv.steering_angle = std::clamp(raw_steer_rate, -params_.max_steer_rate, params_.max_steer_rate);

    // 2. 纵向动力系统一阶滞后与加减速限幅
    const double target_accel_clamped = std::clamp(cmd.target_accel, -params_.max_decel, params_.max_accel);
    deriv.v = (target_accel_clamped - state.accel) / params_.throttle_time_const;
    deriv.accel = deriv.v;

    // 3. 质心侧偏角 beta (Slip angle at center of gravity)
    const double beta = std::atan(params_.lr_ratio * std::tan(state.steering_angle));

    // 4. 运动学位置与航向角导数
    deriv.x = state.v * std::cos(state.theta + beta);
    deriv.y = state.v * std::sin(state.theta + beta);
    deriv.theta = (state.v / params_.wheelbase) * std::cos(beta) * std::tan(state.steering_angle);
    deriv.yaw_rate = deriv.theta;

    return deriv;
}

VehicleState BicycleModel::StepEuler(const VehicleState& current_state, const ControlCommand& cmd,
                                     double dt) const noexcept {
    const VehicleState deriv = ComputeDerivative(current_state, cmd);
    return AccumulateState(current_state, deriv, dt);
}

VehicleState BicycleModel::StepRK4(const VehicleState& current_state, const ControlCommand& cmd,
                                   double dt) const noexcept {
    // 经典 4 阶龙格-库塔 (RK4) 高精度积分
    // k1 = f(x, u)
    const VehicleState k1 = ComputeDerivative(current_state, cmd);

    // k2 = f(x + 0.5*dt*k1, u)
    const VehicleState state_k2 = AccumulateState(current_state, k1, 0.5 * dt);
    const VehicleState k2 = ComputeDerivative(state_k2, cmd);

    // k3 = f(x + 0.5*dt*k2, u)
    const VehicleState state_k3 = AccumulateState(current_state, k2, 0.5 * dt);
    const VehicleState k3 = ComputeDerivative(state_k3, cmd);

    // k4 = f(x + dt*k3, u)
    const VehicleState state_k4 = AccumulateState(current_state, k3, dt);
    const VehicleState k4 = ComputeDerivative(state_k4, cmd);

    // x_{n+1} = x_n + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    VehicleState next_state;
    const double dt_6 = dt / 6.0;

    next_state.x = current_state.x + dt_6 * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x);
    next_state.y = current_state.y + dt_6 * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y);
    next_state.theta =
        NormalizeAngle(current_state.theta + dt_6 * (k1.theta + 2.0 * k2.theta + 2.0 * k3.theta + k4.theta));
    next_state.v = std::clamp(current_state.v + dt_6 * (k1.v + 2.0 * k2.v + 2.0 * k3.v + k4.v), 0.0, params_.max_speed);
    next_state.steering_angle = std::clamp(
        current_state.steering_angle +
            dt_6 * (k1.steering_angle + 2.0 * k2.steering_angle + 2.0 * k3.steering_angle + k4.steering_angle),
        -params_.max_steer_angle, params_.max_steer_angle);
    next_state.accel = current_state.accel + dt_6 * (k1.accel + 2.0 * k2.accel + 2.0 * k3.accel + k4.accel);
    next_state.yaw_rate = (next_state.v / params_.wheelbase) *
                          std::cos(std::atan(params_.lr_ratio * std::tan(next_state.steering_angle))) *
                          std::tan(next_state.steering_angle);

    return next_state;
}

}  // namespace simulation
