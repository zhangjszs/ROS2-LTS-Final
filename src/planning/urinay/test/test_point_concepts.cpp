#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <string>

#include "structures/Point.hpp"
#include "structures/Vector.hpp"

namespace {

struct Custom2DPoint {
    float x;
    float y;
};

struct Custom3DPoint {
    double x;
    double y;
    double z;
};

struct IncompatibleType {
    std::string name;
};

// -------------------------------------------------------------
// 编译期 Concepts 静态断言验证 (Compile-time Concepts Validation)
// -------------------------------------------------------------

// 1. Point2DLike 概念约束
static_assert(urinay::concepts::Point2DLike<Point>, "Point must satisfy Point2DLike");
static_assert(urinay::concepts::Point2DLike<Vector>, "Vector must satisfy Point2DLike");
static_assert(urinay::concepts::Point2DLike<geometry_msgs::msg::Point>, "geometry_msgs::Point must satisfy Point2DLike");
static_assert(urinay::concepts::Point2DLike<geometry_msgs::msg::Point32>, "geometry_msgs::Point32 must satisfy Point2DLike");
static_assert(urinay::concepts::Point2DLike<Custom2DPoint>, "Custom2DPoint must satisfy Point2DLike");
static_assert(urinay::concepts::Point2DLike<Custom3DPoint>, "Custom3DPoint must satisfy Point2DLike");

// 反向断言：非点类型绝不能满足 Point2DLike
static_assert(!urinay::concepts::Point2DLike<int>, "int must NOT satisfy Point2DLike");
static_assert(!urinay::concepts::Point2DLike<std::string>, "std::string must NOT satisfy Point2DLike");
static_assert(!urinay::concepts::Point2DLike<IncompatibleType>, "IncompatibleType must NOT satisfy Point2DLike");

// 2. Point3DLike 概念约束
static_assert(!urinay::concepts::Point3DLike<Point>, "2D Point must NOT satisfy Point3DLike");
static_assert(urinay::concepts::Point3DLike<geometry_msgs::msg::Point>, "geometry_msgs::Point must satisfy Point3DLike");
static_assert(urinay::concepts::Point3DLike<geometry_msgs::msg::Point32>, "geometry_msgs::Point32 must satisfy Point3DLike");
static_assert(urinay::concepts::Point3DLike<Custom3DPoint>, "Custom3DPoint must satisfy Point3DLike");
static_assert(!urinay::concepts::Point3DLike<Custom2DPoint>, "Custom2DPoint must NOT satisfy Point3DLike");

// 3. CalculablePoint 概念约束
static_assert(urinay::concepts::CalculablePoint<Point>, "Point must satisfy CalculablePoint");
static_assert(urinay::concepts::CalculablePoint<geometry_msgs::msg::Point>, "geometry_msgs::Point must satisfy CalculablePoint");
static_assert(urinay::concepts::CalculablePoint<geometry_msgs::msg::Point32>, "geometry_msgs::Point32 must satisfy CalculablePoint");
static_assert(urinay::concepts::CalculablePoint<Custom2DPoint>, "Custom2DPoint must satisfy CalculablePoint");

// 4. ArithmeticScalar 概念约束
static_assert(urinay::concepts::ArithmeticScalar<int>, "int is ArithmeticScalar");
static_assert(urinay::concepts::ArithmeticScalar<float>, "float is ArithmeticScalar");
static_assert(urinay::concepts::ArithmeticScalar<double>, "double is ArithmeticScalar");
static_assert(urinay::concepts::ArithmeticScalar<uint32_t>, "uint32_t is ArithmeticScalar");
static_assert(urinay::concepts::ArithmeticScalar<size_t>, "size_t is ArithmeticScalar");
static_assert(!urinay::concepts::ArithmeticScalar<Point>, "Point is NOT ArithmeticScalar");
static_assert(!urinay::concepts::ArithmeticScalar<std::string>, "std::string is NOT ArithmeticScalar");

}  // namespace

// -------------------------------------------------------------
// 运行时单元测试 (Runtime Unit Tests)
// -------------------------------------------------------------

TEST(PointConcepts, ConstructFromPoint2DLikeTypes) {
    // 从 geometry_msgs::msg::Point 构造
    geometry_msgs::msg::Point gm_pt;
    gm_pt.x = 1.5;
    gm_pt.y = -2.5;
    gm_pt.z = 0.5;
    Point p1(gm_pt);
    EXPECT_DOUBLE_EQ(p1.x, 1.5);
    EXPECT_DOUBLE_EQ(p1.y, -2.5);

    // 从 geometry_msgs::msg::Point32 构造
    geometry_msgs::msg::Point32 gm_pt32;
    gm_pt32.x = 3.25f;
    gm_pt32.y = 4.75f;
    Point p2(gm_pt32);
    EXPECT_NEAR(p2.x, 3.25, 1e-6);
    EXPECT_NEAR(p2.y, 4.75, 1e-6);

    // 从自定义结构体 Custom2DPoint 构造
    Custom2DPoint c2d{10.0f, 20.0f};
    Point p3(c2d);
    EXPECT_DOUBLE_EQ(p3.x, 10.0);
    EXPECT_DOUBLE_EQ(p3.y, 20.0);

    // 从自定义结构体 Custom3DPoint 构造
    Custom3DPoint c3d{7.0, 8.0, 9.0};
    Point p4(c3d);
    EXPECT_DOUBLE_EQ(p4.x, 7.0);
    EXPECT_DOUBLE_EQ(p4.y, 8.0);
}

TEST(PointConcepts, ArithmeticScalarScaling) {
    Point p(2.0, 3.0);

    // 乘法：与各种算术类型相乘
    Point p_int = p * 3;
    EXPECT_DOUBLE_EQ(p_int.x, 6.0);
    EXPECT_DOUBLE_EQ(p_int.y, 9.0);

    Point p_float = p * 0.5f;
    EXPECT_DOUBLE_EQ(p_float.x, 1.0);
    EXPECT_DOUBLE_EQ(p_float.y, 1.5);

    Point p_uint = p * 4u;
    EXPECT_DOUBLE_EQ(p_uint.x, 8.0);
    EXPECT_DOUBLE_EQ(p_uint.y, 12.0);

    // 友元可交换乘法 (scalar * point)
    Point p_comm = 2.5 * p;
    EXPECT_DOUBLE_EQ(p_comm.x, 5.0);
    EXPECT_DOUBLE_EQ(p_comm.y, 7.5);

    // 除法与复合赋值
    Point p_div = p / 2.0;
    EXPECT_DOUBLE_EQ(p_div.x, 1.0);
    EXPECT_DOUBLE_EQ(p_div.y, 1.5);

    p *= 2;
    EXPECT_DOUBLE_EQ(p.x, 4.0);
    EXPECT_DOUBLE_EQ(p.y, 6.0);

    p /= 4.0;
    EXPECT_DOUBLE_EQ(p.x, 1.0);
    EXPECT_DOUBLE_EQ(p.y, 1.5);
}

TEST(PointConcepts, CalculablePointDistance) {
    Point p1(0.0, 0.0);
    Point p2(3.0, 4.0);

    // 点到原点距离（单参重载）
    EXPECT_DOUBLE_EQ(Point::dist(p2), 5.0);
    EXPECT_DOUBLE_EQ(Point::distSq(p2), 25.0);

    // 跨异构点类型的直接几何计算：Point 与 geometry_msgs::Point32
    geometry_msgs::msg::Point32 p32;
    p32.x = 6.0f;
    p32.y = 8.0f;
    EXPECT_DOUBLE_EQ(Point::dist(p1, p32), 10.0);

    // 跨异构点类型：Custom2DPoint 与 Custom3DPoint
    Custom2DPoint c1{1.0f, 1.0f};
    Custom3DPoint c2{4.0, 5.0, 100.0};  // 2D 距离忽略 z
    EXPECT_DOUBLE_EQ(Point::dist(c1, c2), 5.0);
    EXPECT_DOUBLE_EQ(Point::distSq(c1, c2), 25.0);
}

TEST(PointConcepts, GenericToTargetConversion) {
    Point p(4.5, -6.5);

    // 转换为 geometry_msgs::msg::Point
    auto gm_pt = p.to<geometry_msgs::msg::Point>();
    EXPECT_DOUBLE_EQ(gm_pt.x, 4.5);
    EXPECT_DOUBLE_EQ(gm_pt.y, -6.5);
    EXPECT_DOUBLE_EQ(gm_pt.z, 0.0);

    // 转换为 geometry_msgs::msg::Point32
    auto gm_pt32 = p.to<geometry_msgs::msg::Point32>();
    EXPECT_FLOAT_EQ(gm_pt32.x, 4.5f);
    EXPECT_FLOAT_EQ(gm_pt32.y, -6.5f);

    // 转换为自定义点
    auto c2d = p.to<Custom2DPoint>();
    EXPECT_FLOAT_EQ(c2d.x, 4.5f);
    EXPECT_FLOAT_EQ(c2d.y, -6.5f);
}

TEST(PointConcepts, HeterogeneousCcw) {
    Point A(0.0, 0.0);
    geometry_msgs::msg::Point32 B;
    B.x = 2.0f;
    B.y = 0.0f;
    Custom2DPoint C{1.0f, 2.0f};

    // A->B->C 构成逆时针转向
    EXPECT_TRUE(Point::ccw(A, B, C));

    // A->C->B 构成顺时针转向
    EXPECT_FALSE(Point::ccw(A, C, B));
}
