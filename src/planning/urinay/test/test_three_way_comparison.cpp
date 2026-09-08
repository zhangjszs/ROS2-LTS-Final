#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <compare>
#include <limits>
#include <set>
#include <vector>

#include "structures/Circle.hpp"
#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Point.hpp"
#include "structures/Triangle.hpp"
#include "structures/Way.hpp"

// ==============================================================================
// 1. Point 三路比较器与偏序测试 (std::partial_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, PointSpaceshipAndEquality) {
    Point p1{1.0, 2.0};
    Point p2{1.0, 2.0};
    Point p3{1.0, 3.0};
    Point p4{2.0, 1.0};

    // 相等与不等测试（自动重写 == 与 !=）
    EXPECT_TRUE(p1 == p2);
    EXPECT_FALSE(p1 != p2);
    EXPECT_FALSE(p1 == p3);
    EXPECT_TRUE(p1 != p3);

    // 偏序关系测试（默认逐成员字典序比较：先 x 后 y）
    EXPECT_TRUE(p1 < p3);
    EXPECT_TRUE(p1 <= p2);
    EXPECT_TRUE(p1 <= p3);
    EXPECT_TRUE(p4 > p1);
    EXPECT_TRUE(p4 >= p1);

    // 三路操作符返回值类型验证 (std::partial_ordering)
    auto cmp_res = (p1 <=> p2);
    EXPECT_TRUE(cmp_res == std::partial_ordering::equivalent);

    auto cmp_less = (p1 <=> p3);
    EXPECT_TRUE(cmp_less == std::partial_ordering::less);

    auto cmp_greater = (p4 <=> p1);
    EXPECT_TRUE(cmp_greater == std::partial_ordering::greater);
}

TEST(ThreeWayComparisonTest, PointNaNPartialOrdering) {
    const double nan_val = std::numeric_limits<double>::quiet_NaN();
    Point normal_p{1.0, 2.0};
    Point nan_p{nan_val, 2.0};

    // NaN 与任意数比较均为 unordered
    auto cmp_nan = (normal_p <=> nan_p);
    EXPECT_TRUE(cmp_nan == std::partial_ordering::unordered);
    EXPECT_FALSE(normal_p < nan_p);
    EXPECT_FALSE(normal_p > nan_p);
    EXPECT_FALSE(normal_p == nan_p);
    EXPECT_TRUE(normal_p != nan_p);
}

TEST(ThreeWayComparisonTest, PointRangesSort) {
    std::vector<Point> points{{3.0, 1.0}, {1.0, 5.0}, {1.0, 2.0}, {2.0, 0.0}};

    std::ranges::sort(points);

    EXPECT_DOUBLE_EQ(points[0].x, 1.0);
    EXPECT_DOUBLE_EQ(points[0].y, 2.0);
    EXPECT_DOUBLE_EQ(points[1].x, 1.0);
    EXPECT_DOUBLE_EQ(points[1].y, 5.0);
    EXPECT_DOUBLE_EQ(points[2].x, 2.0);
    EXPECT_DOUBLE_EQ(points[2].y, 0.0);
    EXPECT_DOUBLE_EQ(points[3].x, 3.0);
    EXPECT_DOUBLE_EQ(points[3].y, 1.0);
}

// ==============================================================================
// 2. Node 三路比较器与全序测试 (std::strong_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, NodeSpaceshipAndEquality) {
    Node n1(1.0, 2.0, 1.0, 2.0, 10);
    Node n2(5.0, 6.0, 5.0, 6.0, 10);  // 坐标不同但 ID 相同
    Node n3(1.0, 2.0, 1.0, 2.0, 20);

    // Node 比较基于 id
    EXPECT_TRUE(n1 == n2);
    EXPECT_FALSE(n1 != n2);
    EXPECT_TRUE(n1 < n3);
    EXPECT_TRUE(n3 > n1);

    auto cmp_res = (n1 <=> n2);
    static_assert(std::is_same_v<decltype(cmp_res), std::strong_ordering>, "Node <=> must return std::strong_ordering");
    EXPECT_TRUE(cmp_res == std::strong_ordering::equal);
}

// ==============================================================================
// 3. Edge 三路比较器与容器排序测试 (std::strong_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, EdgeSpaceshipAndSet) {
    Node n0(0.0, 0.0, 0.0, 0.0, 1);
    Node n1(1.0, 0.0, 1.0, 0.0, 2);
    Node n2(0.0, 1.0, 0.0, 1.0, 3);

    Edge e01(n0, n1);
    Edge e10(n1, n0);  // 无向边，具有相同的 hash
    Edge e02(n0, n2);

    EXPECT_TRUE(e01 == e10);
    EXPECT_FALSE(e01 != e10);
    EXPECT_TRUE(e01 != e02);

    auto cmp_edge = (e01 <=> e10);
    static_assert(std::is_same_v<decltype(cmp_edge), std::strong_ordering>,
                  "Edge <=> must return std::strong_ordering");
    EXPECT_TRUE(cmp_edge == std::strong_ordering::equal);

    // Edge 支持直接存入 std::set（利用全序 <=> 重写的 operator<）
    std::set<Edge> edge_set;
    edge_set.insert(e01);
    edge_set.insert(e10);  // 重复边应被去重
    edge_set.insert(e02);
    EXPECT_EQ(edge_set.size(), 2u);

    // 关系操作符传递性与互补性测试
    EXPECT_TRUE(e01 < e02 || e02 < e01);
    EXPECT_TRUE((e01 < e02) == (e02 > e01));

    // 指针视图借助投影使用 <=> 比较
    std::vector<const Edge*> edge_ptrs{&e02, &e01};
    std::ranges::sort(edge_ptrs, {}, [](const Edge* ep) { return *ep; });
    EXPECT_TRUE(*edge_ptrs[0] <= *edge_ptrs[1]);
}

// ==============================================================================
// 4. Triangle 三路比较器测试 (std::strong_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, TriangleSpaceshipAndEquality) {
    Node n0(0.0, 0.0, 0.0, 0.0, 1);
    Node n1(1.0, 0.0, 1.0, 0.0, 2);
    Node n2(0.0, 1.0, 0.0, 1.0, 3);
    Node n3(2.0, 2.0, 2.0, 2.0, 4);

    Triangle t1(n0, n1, n2);
    Triangle t2(n2, n0, n1);  // 顶点次序不同但构成同一三角形，hash 相同
    Triangle t3(n0, n1, n3);

    EXPECT_TRUE(t1 == t2);
    EXPECT_FALSE(t1 != t2);
    EXPECT_TRUE(t1 != t3);

    auto cmp_tri = (t1 <=> t2);
    static_assert(std::is_same_v<decltype(cmp_tri), std::strong_ordering>,
                  "Triangle <=> must return std::strong_ordering");
    EXPECT_TRUE(cmp_tri == std::strong_ordering::equal);

    // Triangle 存入 std::set 测试唯一性与全序检索
    std::set<Triangle> tri_set;
    tri_set.insert(t1);
    tri_set.insert(t2);  // 相同 hash 应当被去重
    tri_set.insert(t3);
    EXPECT_EQ(tri_set.size(), 2u);

    EXPECT_TRUE(t1 < t3 || t3 < t1);
    EXPECT_TRUE((t1 < t3) == (t3 > t1));
}

// ==============================================================================
// 5. Way 三路比较器与序列等价测试
// ==============================================================================

TEST(ThreeWayComparisonTest, WaySpaceshipAndEquality) {
    Node n0(0.0, 0.0, 0.0, 0.0, 1);
    Node n1(1.0, 0.0, 1.0, 0.0, 2);
    Node n2(0.0, 1.0, 0.0, 1.0, 3);

    Edge e1(n0, n1);
    Edge e2(n1, n2);

    Way w1;
    w1.addEdge(e1);
    w1.addEdge(e2);

    Way w2;
    w2.addEdge(e1);
    w2.addEdge(e2);

    Way w3;
    w3.addEdge(e1);

    EXPECT_TRUE(w1 == w2);
    EXPECT_FALSE(w1 != w2);
    EXPECT_TRUE(w1 != w3);
    EXPECT_TRUE(w3 < w1);

    auto cmp_way = (w1 <=> w2);
    EXPECT_TRUE(cmp_way == std::strong_ordering::equal);
}

// ==============================================================================
// 6. Circle 三路比较器测试 (std::partial_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, CircleSpaceshipAndEquality) {
    Node n0(0.0, 0.0, 0.0, 0.0, 1);
    Node n1(1.0, 0.0, 1.0, 0.0, 2);
    Node n2(0.0, 1.0, 0.0, 1.0, 3);
    Node n3(2.0, 2.0, 2.0, 2.0, 4);

    Circle c1(n0, n1, n2);
    Circle c2(n0, n1, n2);
    Circle c3(n0, n1, n3);

    EXPECT_TRUE(c1 == c2);
    EXPECT_FALSE(c1 != c2);
    EXPECT_TRUE(c1 != c3);

    auto cmp_res = (c1 <=> c2);
    static_assert(std::is_same_v<decltype(cmp_res), std::partial_ordering>,
                  "Circle <=> must return std::partial_ordering");
    EXPECT_TRUE(cmp_res == std::partial_ordering::equivalent);
}

// ==============================================================================
// 7. Vector 三路比较器测试 (std::partial_ordering)
// ==============================================================================

TEST(ThreeWayComparisonTest, VectorSpaceshipAndEquality) {
    Vector v1{1.0, 2.0};
    Vector v2{1.0, 2.0};
    Vector v3{1.0, 3.0};
    Vector v4{2.0, 1.0};

    EXPECT_TRUE(v1 == v2);
    EXPECT_FALSE(v1 != v2);
    EXPECT_TRUE(v1 < v3);
    EXPECT_TRUE(v4 > v1);

    auto cmp_res = (v1 <=> v2);
    static_assert(std::is_same_v<decltype(cmp_res), std::partial_ordering>,
                  "Vector <=> must return std::partial_ordering");
    EXPECT_TRUE(cmp_res == std::partial_ordering::equivalent);

    auto cmp_less = (v1 <=> v3);
    EXPECT_TRUE(cmp_less == std::partial_ordering::less);
}
