/**
 * @file Vector.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Vector 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Vector.hpp"

/* ----------------------------- 私有方法 ---------------------------- */

/* ----------------------------- 公有方法 ----------------------------- */

double Vector::angleWith(const Vector& v) const {
    double det = this->x * v.y - this->y * v.x;
    return std::atan2(det, this->dot(v));
}