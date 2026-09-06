#pragma once

#include <cmath>

// 把全局位移 (dx, dy) 转到当前车辆 heading 下的 base_link。
inline void GlobalDeltaToBaseLink(double dx, double dy, double theta, double* bx, double* by) {
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    *bx = c * dx + s * dy;
    *by = -s * dx + c * dy;
}
