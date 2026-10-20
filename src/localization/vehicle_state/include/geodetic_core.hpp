#pragma once

// #79：WGS84 大地坐标 → ENU 局部坐标纯 std 核（header-only，无 ROS 依赖）。
//
// 数学逐字搬移自 VehicleStateEstimator::GeodeticToEnu
// （src/localization/vehicle_state/src/vehicle_state_estimator.cpp），节点侧退化为薄转发。
// 行为一致性由一次性差分夹具（位模式对照）证明，见该 Issue 的执行报告。
//
// 单位约定（与原实现一致）：
//   lat / lat0  —— 纬度，弧度
//   lon / lon0  —— 经度，弧度
//   h   / h0    —— 椭球高，米
//   enu_xyz     —— 输出（东, 北, 天），米
//
// 纯数学、无状态、无副作用：同一组输入恒给出同一组输出（确定性）。
// 非有限输入（NaN / inf）按 IEEE-754 语义传播为非有限输出，不做静默钳制、不产生 UB。

#include <cmath>

namespace vehicle_state_core {

// WGS84 椭球参数（与原实现逐字一致：a、b、f、e_sq 的取值完全相同）
inline constexpr double kWgs84SemiMajorAxis = 6378137.0;
inline constexpr double kWgs84SemiMinorAxis = 6356752.3142;
inline constexpr double kWgs84Flattening = (kWgs84SemiMajorAxis - kWgs84SemiMinorAxis) / kWgs84SemiMajorAxis;
inline constexpr double kWgs84EccentricitySq = kWgs84Flattening * (2 - kWgs84Flattening);

// 大地坐标 → 地心地固（ECEF）坐标，单位米。
inline void EcefCore(double la, double lo, double alt, double& X, double& Y, double& Z) {
    const double s = std::sin(la);
    const double N = kWgs84SemiMajorAxis / std::sqrt(1 - kWgs84EccentricitySq * s * s);
    X = (alt + N) * std::cos(la) * std::cos(lo);
    Y = (alt + N) * std::cos(la) * std::sin(lo);
    Z = (alt + (1 - kWgs84EccentricitySq) * N) * std::sin(la);
}

// 大地坐标 → 以 (lat0, lon0, h0) 为原点的 ENU 局部坐标，单位米。
inline void GeodeticToEnuCore(double lat, double lon, double h, double lat0, double lon0, double h0,
                              double enu_xyz[3]) {
    double x, y, z, x0, y0, z0;
    EcefCore(lat, lon, h, x, y, z);
    EcefCore(lat0, lon0, h0, x0, y0, z0);

    const double xd = x - x0, yd = y - y0, zd = z - z0;
    const double t = -std::cos(lon0) * xd - std::sin(lon0) * yd;

    enu_xyz[0] = -std::sin(lon0) * xd + std::cos(lon0) * yd;
    enu_xyz[1] = t * std::sin(lat0) + std::cos(lat0) * zd;
    enu_xyz[2] = std::cos(lat0) * std::cos(lon0) * xd + std::cos(lat0) * std::sin(lon0) * yd + std::sin(lat0) * zd;
}

}  // namespace vehicle_state_core
