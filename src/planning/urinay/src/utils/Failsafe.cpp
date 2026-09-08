/**
 * @file Failsafe.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Failsafe 类成员函数的实现
 * @version 1.0
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2023 BCN eMotorsport
 */

#include "utils/Failsafe.hpp"

#include <numbers>

/* ----------------------------- 私有方法 ---------------------------- */

/* ----------------------------- 公有方法  ---------------------------- */

template <typename T>
void Failsafe<T>::initGeneral(const T& params, const double& safetyFactor, const int& failsafe_max_way_horizon_size) {
    *this = Failsafe<T>(params);
    this->max_way_horizon_size = failsafe_max_way_horizon_size;

    this->search_radius *= safetyFactor;
    constexpr double kHalfPi = std::numbers::pi_v<double> / 2.0;
    this->max_angle_diff = std::min(this->max_angle_diff * safetyFactor, kHalfPi);
    this->edge_len_diff_factor *= safetyFactor;
    this->max_next_heuristic *= safetyFactor;
}
template void Failsafe<UrinayParams::WayComputer::Search>::initGeneral(const UrinayParams::WayComputer::Search&,
                                                                       const double&, const int&);