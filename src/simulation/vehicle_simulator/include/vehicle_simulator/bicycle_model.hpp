#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <numbers>

namespace simulation {

/**
 * @brief 车辆运动状态
 */
struct VehicleState {
    double x{0.0};               // 全局 X 坐标 (m)
    double y{0.0};               // 全局 Y 坐标 (m)
    double theta{0.0};           // 航向角 (rad)
    double v{0.0};               // 车速 (m/s)
    double steering_angle{0.0};  // 前轮转角 (rad)
    double accel{0.0};           // 纵向加速度 (m/s^2)
    double yaw_rate{0.0};        // 横摆角速度 (rad/s)
};

/**
 * @brief 控制输入指令
 */
struct ControlCommand {
    double target_steering{0.0};  // 目标前轮转角 (rad)
    double target_accel{0.0};     // 目标纵向加速度 (m/s^2)
};

/**
 * @brief 赛车动力学与几何参数 (默认匹配 FSAC 电动方程式赛车)
 */
struct VehicleParams {
    double wheelbase{1.55};            // 轴距 L = l_f + l_r (m)
    double lr_ratio{0.50};             // 后轴到质心距离占比 l_r / L
    double max_steer_angle{0.40};      // 最大转向角 (rad, 约 23 度)
    double max_steer_rate{3.0};        // 最大转向角速度 (rad/s)
    double max_accel{6.0};             // 最大纵向加速度 (m/s^2, 约 0.6g)
    double max_decel{9.0};             // 最大制动减速度 (m/s^2, 约 0.9g)
    double max_speed{25.0};            // 最大巡航车速 (m/s, 90 km/h)
    double steer_time_const{0.05};     // 转向执行机构一阶延迟时间常数 (s)
    double throttle_time_const{0.10};  // 动力系统油门一阶响应时间常数 (s)
    double roll_gain{0.04};            // 侧向加速度转侧倾角增益
    double pitch_gain{0.03};           // 纵向加速度转俯仰角增益
};

/**
 * @brief C++20 概念：数值积分器约束
 */
template <typename T>
concept IntegratorConcept =
    requires(T integrator, const VehicleState& s, const ControlCommand& u, const VehicleParams& p, double dt) {
        { integrator(s, u, p, dt) } -> std::same_as<VehicleState>;
    };

/**
 * @brief 单轨运动学自行车模型 (Kinematic Bicycle Model)
 */
class BicycleModel {
   public:
    explicit BicycleModel(const VehicleParams& params = VehicleParams{}) : params_(params) {}

    /**
     * @brief 计算微分方程导数项: dx/dt = f(x, u)
     */
    [[nodiscard]] VehicleState ComputeDerivative(const VehicleState& state, const ControlCommand& cmd) const noexcept;

    /**
     * @brief 4阶龙格-库塔 (RK4) 高精度数值单步积分
     */
    [[nodiscard]] VehicleState StepRK4(const VehicleState& current_state, const ControlCommand& cmd,
                                       double dt) const noexcept;

    /**
     * @brief 欧拉法快速积分
     */
    [[nodiscard]] VehicleState StepEuler(const VehicleState& current_state, const ControlCommand& cmd,
                                         double dt) const noexcept;

    /**
     * @brief 重置车辆状态
     */
    void Reset(double x = 0.0, double y = 0.0, double theta = 0.0, double v = 0.0) noexcept {
        state_ = VehicleState{.x = x, .y = y, .theta = theta, .v = v, .steering_angle = 0.0, .accel = 0.0};
    }

    /**
     * @brief 状态推进 (默认采用 RK4)
     */
    const VehicleState& Step(const ControlCommand& cmd, double dt) noexcept {
        state_ = StepRK4(state_, cmd, dt);
        return state_;
    }

    [[nodiscard]] const VehicleState& state() const noexcept { return state_; }
    [[nodiscard]] const VehicleParams& params() const noexcept { return params_; }
    void set_params(const VehicleParams& p) noexcept { params_ = p; }

   private:
    VehicleParams params_;
    VehicleState state_{};
};

}  // namespace simulation
