/**
 * @file Point.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief 包含 Point 类的定义
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

/**
 * @brief 表示二维空间中的一个点，并包含有用的工具来执行
 * 相关操作，也可以将其转换为 ROS 消息。
 */
class Point {
   public:
    /* -------------------------- 公共构造函数 ------------------------- */

    Point();
    Point(const double &x, const double &y);
    template <typename T>
    Point(const T &point);

    /* --------------------------- 公共属性 -------------------------- */

    double x, y;

    /* ---------------------------- 公共方法 ---------------------------- */

    Point operator+(const Point &p) const;
    Point operator-(const Point &p) const;

    template <typename T>
    Point operator*(const T &num) const;

    template <typename T>
    Point operator/(const T &num) const;

    Point &operator+=(const Point &p);
    Point &operator-=(const Point &p);

    template <typename T>
    Point &operator*=(const T &num);

    template <typename T>
    Point &operator/=(const T &num);

    /**
     * @brief 计算两个点之间的平方欧几里得距离。
     *
     * @param[in] p1
     * @param[in] p2
     */
    static inline double distSq(const Point &p1, const Point &p2 = Point()) {
        const double dx = p1.x - p2.x;
        const double dy = p1.y - p2.y;
        return dx * dx + dy * dy;
    }

    /**
     * @brief 计算两个点之间的欧几里得距离。
     *
     * @param[in] p1
     * @param[in] p2
     */
    static inline double dist(const Point &p1, const Point &p2 = Point()) {
        const double dx = p1.x - p2.x;
        const double dy = p1.y - p2.y;
        return sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief 检查有序三元组 (A, B, C) 的方向是否为逆时针。
     *
     * @param[in] A
     * @param[in] B
     * @param[in] C
     */
    static inline bool ccw(const Point &A, const Point &B, const Point &C) {
        return (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);
    }

    /**
     * @brief 打印一个点。
     *
     * @param[in,out] os
     * @param[in] p
     */
    friend std::ostream &operator<<(std::ostream &os, const Point &p);

    /**
     * @brief 使用 Eigen::Affine3d 返回变换后的点。
     *
     * @param[in] tf
     */
    Point transformed(const Eigen::Affine3d &tf) const;

    /**
     * @brief 将点转换为 geometry_msgs::Point。
     */
    geometry_msgs::msg::Point gmPoint() const;

    /**
     * @brief 返回由 ind 指定的位置的坐标。
     *
     * @param[in] ind
     */
    const double &at(const size_t &ind) const;

    /**
     * @brief 返回 2。
     */
    size_t size() const;
};
