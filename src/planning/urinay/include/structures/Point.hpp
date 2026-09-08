/**
 * @file Point.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Point 类的定义与 C++20 空间点概念（Concepts）
 * @version 2.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <compare>
#include <concepts>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>
#include <type_traits>

namespace urinay::concepts {

/**
 * @brief 标量算术类型概念：约束可以与几何坐标进行乘除缩放的算术类型。
 */
template <typename T>
concept ArithmeticScalar = std::is_arithmetic_v<std::remove_cvref_t<T>>;

/**
 * @brief 二维空间点概念：必须具备可转换为 double 的 x 与 y 成员坐标。
 */
template <typename T>
concept Point2DLike = requires(const T& p) {
    { p.x } -> std::convertible_to<double>;
    { p.y } -> std::convertible_to<double>;
};

/**
 * @brief 三维空间点概念：在具备 x, y 坐标基础上，同时具备可转换为 double 的 z 坐标。
 */
template <typename T>
concept Point3DLike = Point2DLike<T> && requires(const T& p) {
    { p.z } -> std::convertible_to<double>;
};

/**
 * @brief 空间几何点顶层概念：满足二维空间点特征的几何类型。
 */
template <typename T>
concept SpatialPoint = Point2DLike<T>;

/**
 * @brief 可计算空间几何点概念：约束可参与欧几里得距离度量与坐标差值计算的点类型。
 */
template <typename T>
concept CalculablePoint = SpatialPoint<T> && requires(const T& p1, const T& p2) {
    { static_cast<double>(p1.x) - static_cast<double>(p2.x) } -> std::convertible_to<double>;
    { static_cast<double>(p1.y) - static_cast<double>(p2.y) } -> std::convertible_to<double>;
};

}  // namespace urinay::concepts

/**
 * @brief 表示二维空间中的一个点，并包含有用的工具来执行
 * 相关操作，也可以将其转换为 ROS 消息。
 */
class Point {
   public:
    /* -------------------------- 公共构造函数 ------------------------- */

    constexpr Point() noexcept : x(0.0), y(0.0) {}
    constexpr Point(double x_val, double y_val) noexcept : x(x_val), y(y_val) {}

    /**
     * @brief 使用 C++20 Concept Point2DLike 泛型约束构造函数。
     * 支持从任意包含 x, y 成员的类型（如 geometry_msgs::Point, Point32, PCL 点等）无缝泛型构造。
     */
    template <urinay::concepts::Point2DLike T>
        requires(!std::same_as<std::remove_cvref_t<T>, Point>)
    constexpr Point(const T& point) noexcept : x(static_cast<double>(point.x)), y(static_cast<double>(point.y)) {}

    /* --------------------------- 公共属性 -------------------------- */

    double x{0.0}, y{0.0};

    /* -------------------------- 比较运算符 (C++20) ------------------------- */
    /**
     * @brief C++20 宇宙飞船操作符 <=> 默认合成。
     * 由于包含 double 成员，返回类型为 std::partial_ordering（处理 NaN 偏序）。
     * 自动合成 <, <=, >, >=。
     */
    [[nodiscard]] constexpr auto operator<=>(const Point&) const noexcept = default;

    /**
     * @brief C++20 默认相等比较操作符，自动合成 == 与 !=。
     */
    [[nodiscard]] constexpr bool operator==(const Point&) const noexcept = default;

    /* ---------------------------- 公共方法 ---------------------------- */

    [[nodiscard]] constexpr Point operator+(const Point& p) const noexcept {
        return Point(this->x + p.x, this->y + p.y);
    }

    [[nodiscard]] constexpr Point operator-(const Point& p) const noexcept {
        return Point(this->x - p.x, this->y - p.y);
    }

    template <urinay::concepts::ArithmeticScalar T>
    [[nodiscard]] constexpr Point operator*(const T& num) const noexcept {
        return Point(this->x * static_cast<double>(num), this->y * static_cast<double>(num));
    }

    template <urinay::concepts::ArithmeticScalar T>
    [[nodiscard]] constexpr Point operator/(const T& num) const noexcept {
        return Point(this->x / static_cast<double>(num), this->y / static_cast<double>(num));
    }

    constexpr Point& operator+=(const Point& p) noexcept {
        this->x += p.x;
        this->y += p.y;
        return *this;
    }

    constexpr Point& operator-=(const Point& p) noexcept {
        this->x -= p.x;
        this->y -= p.y;
        return *this;
    }

    template <urinay::concepts::ArithmeticScalar T>
    constexpr Point& operator*=(const T& num) noexcept {
        this->x *= static_cast<double>(num);
        this->y *= static_cast<double>(num);
        return *this;
    }

    template <urinay::concepts::ArithmeticScalar T>
    constexpr Point& operator/=(const T& num) noexcept {
        this->x /= static_cast<double>(num);
        this->y /= static_cast<double>(num);
        return *this;
    }

    template <urinay::concepts::ArithmeticScalar T>
    friend constexpr Point operator*(const T& num, const Point& p) noexcept {
        return p * num;
    }

    /**
     * @brief 计算两个点之间的平方欧几里得距离（支持任意满足 CalculablePoint 的异构几何类型）。
     */
    template <urinay::concepts::CalculablePoint P1, urinay::concepts::CalculablePoint P2 = Point>
    static inline double distSq(const P1& p1, const P2& p2 = Point()) noexcept {
        const double dx = static_cast<double>(p1.x) - static_cast<double>(p2.x);
        const double dy = static_cast<double>(p1.y) - static_cast<double>(p2.y);
        return dx * dx + dy * dy;
    }

    /**
     * @brief 计算两个点之间的欧几里得距离（支持任意满足 CalculablePoint 的异构几何类型）。
     */
    template <urinay::concepts::CalculablePoint P1, urinay::concepts::CalculablePoint P2 = Point>
    static inline double dist(const P1& p1, const P2& p2 = Point()) noexcept {
        return std::sqrt(distSq(p1, p2));
    }

    /**
     * @brief 检查有序三元组 (A, B, C) 的方向是否为逆时针。
     */
    template <urinay::concepts::CalculablePoint P1, urinay::concepts::CalculablePoint P2,
              urinay::concepts::CalculablePoint P3>
    static inline bool ccw(const P1& A, const P2& B, const P3& C) noexcept {
        return (static_cast<double>(C.y) - static_cast<double>(A.y)) *
                   (static_cast<double>(B.x) - static_cast<double>(A.x)) >
               (static_cast<double>(B.y) - static_cast<double>(A.y)) *
                   (static_cast<double>(C.x) - static_cast<double>(A.x));
    }

    /**
     * @brief 打印一个点。
     *
     * @param[in,out] os
     * @param[in] p
     */
    friend std::ostream& operator<<(std::ostream& os, const Point& p);

    /**
     * @brief 使用 Eigen::Affine3d 返回变换后的点。
     *
     * @param[in] tf
     */
    Point transformed(const Eigen::Affine3d& tf) const;

    /**
     * @brief 将点转换为 geometry_msgs::Point。
     */
    geometry_msgs::msg::Point gmPoint() const;

    /**
     * @brief 泛型转换为满足 Point2DLike 的目标点类型。
     */
    template <urinay::concepts::Point2DLike Target = geometry_msgs::msg::Point>
    Target to() const noexcept {
        Target res{};
        res.x = static_cast<decltype(res.x)>(this->x);
        res.y = static_cast<decltype(res.y)>(this->y);
        return res;
    }

    /**
     * @brief 返回由 ind 指定的位置的坐标（0 为 x，其他为 y）。
     */
    [[nodiscard]] constexpr double at(size_t ind) const noexcept { return (ind == 0) ? this->x : this->y; }

    /**
     * @brief 返回点维度（2）。
     */
    [[nodiscard]] static constexpr size_t size() noexcept { return 2; }
};
