/**
 * @file Failsafe.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Failsafe 类的定义
 * @version 1.0
 * @date 2022-05-06
 *
 * @copyright Copyright (c) 2023 BCN eMotorsport
 */

#pragma once

#include <cmath>

#include "utils/constants.hpp"
#include "utils/urinay_params.hpp"

/**
 * @brief 提供创建和管理故障保护（Failsafe）参数的所有功能的类。
 */
template <typename T>
class Failsafe : public T {
   private:
   public:
    Failsafe() = default;
    Failsafe(const T& x) : T(x) {}

    /**
     * @brief 将隐式对象设置为通用故障保护对象，使用其参数。
     * 目标是：
     * - 将所有参数（阈值）按一个因子增加。
     * - 限制视野长度。
     *
     * @param params
     * @param safetyFactor
     * @param failsafe_max_way_horizon_size
     */
    void initGeneral(const T& params, const double& safetyFactor, const int& failsafe_max_way_horizon_size);
};
