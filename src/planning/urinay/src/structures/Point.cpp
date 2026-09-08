/**
 * @file Point.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Point 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Point.hpp"

std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << "P(" << p.x << ", " << p.y << ")\n";
}

// 实现将点从一个坐标系变换到另一个坐标系的功能，返回变换后的点
Point Point::transformed(const Eigen::Affine3d& tf) const {
    Eigen::Vector3d product = tf * Eigen::Vector3d(this->x, this->y, 0.0);
    return Point(product.x(), product.y());
}

geometry_msgs::msg::Point Point::gmPoint() const {
    geometry_msgs::msg::Point res;
    res.x = this->x;
    res.y = this->y;
    res.z = 0.0;
    return res;
}