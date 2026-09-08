/**
 * @file Node.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Node 类成员函数的实现
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Node.hpp"

const uint32_t Node::SUPERTRIANGLE_BASEID;
uint32_t Node::superTriangleNodeNum = 0;

/* ----------------------------- 私有方法 ---------------------------- */

Node::Node(double x, double y)
    : id(SUPERTRIANGLE_BASEID + superTriangleNodeNum), point_(x, y), belongsToSuperTriangle_(true) {
    superTriangleNodeNum++;
    superTriangleNodeNum %= 3;
}

/* ----------------------------- 公有方法 ----------------------------- */

Node::Node(double x, double y, double xGlobal, double yGlobal, uint32_t id)
    : id(id), point_(x, y), pointGlobal_(xGlobal, yGlobal), belongsToSuperTriangle_(false) {
    if (this->id >= (1 << HASH_SHIFT_NUM) - 3)
        RCLCPP_ERROR(rclcpp::get_logger("urinay"), "[urinay] Cone ID exceeds allowed threshold, check utils/constants.hpp/HASH_SHIFT_NUM");
}

Node::Node(const common_msgs::msg::HuatCone &c)
    : Node(c.position_base_link.x, c.position_base_link.y, c.position_global.x, c.position_global.y, c.id) {}

double Node::x() const noexcept {
    return this->point_.x;
}

double Node::y() const noexcept {
    return this->point_.y;
}

Node Node::superTriangleNode(double x, double y) {
    return Node(x, y);
}

bool Node::belongsToSuperTriangle() const noexcept {
    return belongsToSuperTriangle_;
}

void Node::updateLocal(const Eigen::Affine3d &tf) const {
    this->point_ = this->pointGlobal().transformed(tf);
}

const Point &Node::point() const noexcept {
    return this->point_;
}

const Point &Node::pointGlobal() const noexcept {  //构造函数可直接赋值
    return this->pointGlobal_;
}

double Node::distSq(const Point &p) const noexcept {
    return (this->x() - p.x) * (this->x() - p.x) + (this->y() - p.y) * (this->y() - p.y);
}

double Node::angleWith(const Node &n0, const Node &n1) const {
    return std::abs(Vector(this->point(), n0.point()).angleWith(Vector(this->point(), n1.point())));
}

common_msgs::msg::HuatCone Node::cone() const {
    common_msgs::msg::HuatCone res;
    res.id = this->id;
    res.position_global.x = this->pointGlobal().gmPoint().x;
    res.position_global.y = this->pointGlobal().gmPoint().y;
    res.position_base_link.x = this->point().gmPoint().x;
    res.position_base_link.y = this->point().gmPoint().y;

    res.type = huat_cone::NONE;
    return res;
}

std::ostream &operator<<(std::ostream &os, const Node &n) {
    os << "N(" << n.x() << ", " << n.y() << ")";
    return os;
}