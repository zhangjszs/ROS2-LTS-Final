#pragma once
// ============================================================================
// issue #29：离线 runner 速度包络单一来源（纯 std，无 ROS 依赖）。
//
// benchmark_runner 曾在两处各自内联同一条曲率限速式（PP 分支的 v_curve 与
// build_mpc_reference 的逐点速度）：“同一剖面输入、两种写法”——改一处漏一处
// 即制造控制器间不公平。本头是 runner 内速度包络的唯一实现：PP 分支与 MPC
// 参考构建均调同一函数，identical input → identical envelope（#29 验收第 1 条
// 在 runner 侧的机读载体）。
//
// 口径说明：本函数是“曲率限速 + 固定上限”包络（与 v1 基线同式），不是
// velocity_profiler 的 G-G/DP 全剖面。把 runner 切到 profiler DP 会改变速度
// 包络、触发基线重录程序（benchmark_regression.sh §6），留 #19 B5 与基线
// 程序按流程做，本轮只收敛“同一口径、同一实现”。
// ============================================================================
#include <algorithm>
#include <cmath>

namespace benchmark {

/// 曲率限速包络：v = min(v_max, sqrt(a_lat_max / max(|kappa|, 1e-3)))。
/// kappa 取幅值（转向方向不影响限速）；直道（kappa≈0）给 v_max。
[[nodiscard]] inline double referenceSpeedForCurvature(double kappa_abs, double lat_accel_max, double v_max) noexcept {
    const double k = std::abs(kappa_abs);
    return std::min(v_max, std::sqrt(lat_accel_max / std::max(k, 1e-3)));
}

}  // namespace benchmark
