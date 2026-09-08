/**
 * @file Circle.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Circle 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Circle.hpp"

/* ----------------------------- 私有方法 ---------------------------- */

/* ----------------------------- 公有方法 ----------------------------- */

Circle::Circle(const Node &p0, const Node &p1, const Node &p2) {
    // 查找 base_link 坐标系下的圆心
    const double ax = p1.x() - p0.x();
    const double ay = p1.y() - p0.y();
    const double bx = p2.x() - p0.x();
    const double by = p2.y() - p0.y();

    const double m = p1.x() * p1.x() - p0.x() * p0.x() + p1.y() * p1.y() - p0.y() * p0.y();
    const double u = p2.x() * p2.x() - p0.x() * p0.x() + p2.y() * p2.y() - p0.y() * p0.y();
    const double s = 1. / (2. * (ax * by - ay * bx));

    this->center_.x = ((p2.y() - p0.y()) * m + (p0.y() - p1.y()) * u) * s;
    this->center_.y = ((p0.x() - p2.x()) * m + (p1.x() - p0.x()) * u) * s;

    // 查找全局坐标系下的圆心
    const double axGlobal = p1.pointGlobal().x - p0.pointGlobal().x;
    const double ayGlobal = p1.pointGlobal().y - p0.pointGlobal().y;
    const double bxGlobal = p2.pointGlobal().x - p0.pointGlobal().x;
    const double byGlobal = p2.pointGlobal().y - p0.pointGlobal().y;

    const double mGlobal = p1.pointGlobal().x * p1.pointGlobal().x - p0.pointGlobal().x * p0.pointGlobal().x +
                           p1.pointGlobal().y * p1.pointGlobal().y - p0.pointGlobal().y * p0.pointGlobal().y;
    const double uGlobal = p2.pointGlobal().x * p2.pointGlobal().x - p0.pointGlobal().x * p0.pointGlobal().x +
                           p2.pointGlobal().y * p2.pointGlobal().y - p0.pointGlobal().y * p0.pointGlobal().y;
    const double sGlobal = 1. / (2. * (axGlobal * byGlobal - ayGlobal * bxGlobal));

    this->centerGlobal_.x =
        ((p2.pointGlobal().y - p0.pointGlobal().y) * mGlobal + (p0.pointGlobal().y - p1.pointGlobal().y) * uGlobal) *
        sGlobal;
    this->centerGlobal_.y =
        ((p0.pointGlobal().x - p2.pointGlobal().x) * mGlobal + (p1.pointGlobal().x - p0.pointGlobal().x) * uGlobal) *
        sGlobal;

    const double dx = p0.x() - this->center_.x;
    const double dy = p0.y() - this->center_.y;
    this->radSq_ = dx * dx + dy * dy;
}

bool Circle::containsNode(const Node &n) const {
    return n.distSq(this->center_) < this->radSq_;
}

const Point &Circle::center() const noexcept {
    return this->center_;
}

const Point &Circle::centerGlobal() const noexcept {
    return this->centerGlobal_;
}
