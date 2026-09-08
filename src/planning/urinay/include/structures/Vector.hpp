/**
 * @file Vector.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Vector 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include "structures/Point.hpp"

/**
 * @brief 表示二维空间中的一个向量，继承自 Point 类。
 * 它提供了二维空间中向量的所有基本功能。
 */
class Vector : public Point {
   public:
    /**
     * @brief 构造一个新的 Vector 对象。
     */
    constexpr Vector() noexcept : Point() {}

    [[nodiscard]] constexpr auto operator<=>(const Vector &) const noexcept = default;
    [[nodiscard]] constexpr bool operator==(const Vector &) const noexcept = default;

    /**
     * @brief 从两个点构造一个新的 Vector 对象。
     *
     * @param[in] a
     * @param[in] b
     */
    constexpr Vector(const Point &a, const Point &b) noexcept : Point(b - a) {}

    /**
     * @brief 从两个坐标构造一个新的 Vector 对象。
     *
     * @param[in] x
     * @param[in] y
     */
    constexpr Vector(double x_val, double y_val) noexcept : Point(x_val, y_val) {}

    /**
     * @brief 返回隐式向量与 v 之间的点积。
     *
     * @param[in] v
     */
    [[nodiscard]] constexpr double dot(const Vector &v) const noexcept {
        return this->x * v.x + this->y * v.y;
    }

    /**
     * @brief 返回隐式向量与 v 之间的夹角。
     *
     * @param[in] v
     */
    [[nodiscard]] double angleWith(const Vector &v) const;

    /**
     * @brief 检查点 actPos 是否沿着方向 dir 在 futPos 的后方。
     *
     * @param[in] futPos
     * @param[in] actPos
     * @param[in] dir
     */
    [[nodiscard]] static constexpr bool pointBehind(const Point &futPos, const Point &actPos, const Vector &dir) noexcept {
        return Vector(actPos, futPos).dot(dir) < 0.0;
    }

    /**
     * @brief 返回顺时针旋转 90 度的向量。
     */
    [[nodiscard]] constexpr Vector rotClock() const noexcept {
        return Vector(this->y, -this->x);
    }

    /**
     * @brief 返回逆时针旋转 90 度的向量。
     */
    [[nodiscard]] constexpr Vector rotCounterClock() const noexcept {
        return Vector(-this->y, this->x);
    }
};
