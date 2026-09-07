#include <gtest/gtest.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <ranges>
#include <vector>

#include "structures/Point.hpp"
#include "utils/KDTree.hpp"

namespace {

struct CustomConePoint {
    float x;
    float y;
    int id;
};

// -------------------------------------------------------------
// 编译期 Concepts 静态断言验证 (Compile-time Concepts Validation)
// -------------------------------------------------------------

// 1. DistanceMetric 概念约束
static_assert(urinay::concepts::DistanceMetric<SquaredEuclideanMetric, Point>, "SquaredEuclideanMetric must satisfy DistanceMetric for Point");
static_assert(urinay::concepts::DistanceMetric<SquaredEuclideanMetric, geometry_msgs::msg::Point32>, "SquaredEuclideanMetric must satisfy DistanceMetric for Point32");
static_assert(urinay::concepts::DistanceMetric<ManhattanMetric, Point>, "ManhattanMetric must satisfy DistanceMetric for Point");
static_assert(urinay::concepts::DistanceMetric<ManhattanMetric, CustomConePoint>, "ManhattanMetric must satisfy DistanceMetric for CustomConePoint");
static_assert(!urinay::concepts::DistanceMetric<int, Point>, "int must NOT satisfy DistanceMetric");

// 2. SpatialIndexablePoint 概念约束
static_assert(urinay::concepts::SpatialIndexablePoint<Point>, "Point must satisfy SpatialIndexablePoint");
static_assert(urinay::concepts::SpatialIndexablePoint<geometry_msgs::msg::Point>, "geometry_msgs::Point must satisfy SpatialIndexablePoint");
static_assert(urinay::concepts::SpatialIndexablePoint<geometry_msgs::msg::Point32>, "geometry_msgs::Point32 must satisfy SpatialIndexablePoint");
static_assert(urinay::concepts::SpatialIndexablePoint<CustomConePoint>, "CustomConePoint must satisfy SpatialIndexablePoint");
static_assert(!urinay::concepts::SpatialIndexablePoint<double>, "double must NOT satisfy SpatialIndexablePoint");

}  // namespace

// -------------------------------------------------------------
// 运行时单元测试 (Runtime Unit Tests)
// -------------------------------------------------------------

TEST(KDTreeConcepts, DefaultKDTreeBackwardCompatibility) {
    std::vector<Point> pts = {
        Point(0.0, 0.0),
        Point(2.0, 0.0),
        Point(0.0, 2.0),
        Point(2.0, 2.0)
    };

    KDTree tree(pts);
    EXPECT_FALSE(tree.empty());
    EXPECT_EQ(tree.size(), 4u);

    // 查询最近邻
    Point q(0.1, 0.2);
    auto nearest = tree.nearest_point(q);
    ASSERT_TRUE(static_cast<bool>(nearest));
    EXPECT_DOUBLE_EQ(nearest->x, 0.0);
    EXPECT_DOUBLE_EQ(nearest->y, 0.0);

    // 带排除集合的查询 (排除索引 0)
    std::set<size_t> excs = {0};
    auto nearest_ex = tree.nearest_index(q, excs);
    ASSERT_TRUE(static_cast<bool>(nearest_ex));
    EXPECT_NE(*nearest_ex, 0u);

    // 半径查询
    auto neighbors = tree.neighborhood_indices(Point(0.0, 0.0), 2.1);
    EXPECT_EQ(neighbors.size(), 3u);  // (0,0), (2,0), (0,2) 在半径 2.1 内
}

TEST(KDTreeConcepts, GenericRosPoint32Tree) {
    // 直接使用 ROS 消息类型 geometry_msgs::msg::Point32 建树，零类型转换
    std::vector<geometry_msgs::msg::Point32> pts(4);
    pts[0].x = 10.0f; pts[0].y = 10.0f;
    pts[1].x = 20.0f; pts[1].y = 10.0f;
    pts[2].x = 10.0f; pts[2].y = 20.0f;
    pts[3].x = 20.0f; pts[3].y = 20.0f;

    BasicKDTree<geometry_msgs::msg::Point32> tree(pts);
    EXPECT_EQ(tree.size(), 4u);

    // 查询可跨类型使用 Point 或 Point32
    Point query_pt(19.0, 19.5);
    auto nearest = tree.nearest_point(query_pt);
    ASSERT_TRUE(static_cast<bool>(nearest));
    EXPECT_FLOAT_EQ(nearest->x, 20.0f);
    EXPECT_FLOAT_EQ(nearest->y, 20.0f);

    auto nearest_idx = tree.nearest_index(query_pt);
    ASSERT_TRUE(static_cast<bool>(nearest_idx));
    EXPECT_EQ(*nearest_idx, 3u);
}

TEST(KDTreeConcepts, ManhattanMetricTree) {
    // 测试使用曼哈顿距离度量的 KDTree
    std::vector<Point> pts = {
        Point(0.0, 0.0),
        Point(3.0, 0.0),
        Point(0.0, 4.0),
        Point(2.0, 2.0)
    };

    BasicKDTree<Point, ManhattanMetric> tree(pts);

    // 查询 (1.0, 1.0):
    // 到 (0,0) 的曼哈顿距离 = |1-0| + |1-0| = 2
    // 到 (2,2) 的曼哈顿距离 = |1-2| + |1-2| = 2
    // 到 (3,0) 的曼哈顿距离 = |1-3| + |1-0| = 3
    Point q(1.0, 1.0);
    auto nearest = tree.nearest_point(q);
    ASSERT_TRUE(static_cast<bool>(nearest));
    double dist = ManhattanMetric{}(*nearest, q);
    EXPECT_DOUBLE_EQ(dist, 2.0);

    // 曼哈顿半径查询：radius = 2.5
    // 只有 (0,0) [d=2] 与 (2,2) [d=2] 满足条件
    auto indices = tree.neighborhood_indices(q, 2.5);
    EXPECT_EQ(indices.size(), 2u);
}

TEST(KDTreeConcepts, RangeViewConstruction) {
    std::vector<Point> all_pts = {
        Point(-5.0, 0.0),
        Point(1.0, 2.0),
        Point(3.0, 4.0),
        Point(-10.0, 5.0),
        Point(5.0, 6.0)
    };

    // 使用 C++20 views::filter 惰性筛选 x > 0 的正半轴点建树，消灭中间临时容器
    auto positive_x = all_pts | std::views::filter([](const Point& p) { return p.x > 0.0; });
    KDTree tree(positive_x);

    EXPECT_EQ(tree.size(), 3u);

    Point q(2.5, 3.5);
    auto nearest = tree.nearest_point(q);
    ASSERT_TRUE(static_cast<bool>(nearest));
    EXPECT_DOUBLE_EQ(nearest->x, 3.0);
    EXPECT_DOUBLE_EQ(nearest->y, 4.0);
}
