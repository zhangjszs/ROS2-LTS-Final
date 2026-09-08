#include <gtest/gtest.h>

#include "structures/Way.hpp"

TEST(WayEmpty, DefaultIsEmpty) {
    Way way;
    EXPECT_TRUE(way.empty());
    EXPECT_EQ(way.size(), 0u);
}

TEST(WayEmpty, InterpolationDoesNotCrash) {
    Way way;
    EXPECT_TRUE(way.getPathInterpolation(0.0, 0.0).empty());
    EXPECT_TRUE(way.getPathInterpolationLocal(0.0, 0.0).empty());
    EXPECT_TRUE(way.getPathFullInterpolation().empty());
    EXPECT_TRUE(way.getPathFullInterpolationLocal().empty());
}

TEST(WayEmpty, DeletePassedIsNoOp) {
    Way way;
    way.deleteWayPassed();
    EXPECT_TRUE(way.empty());
}

TEST(WayEmpty, NextPathPointIsOrigin) {
    Way way;
    Point p = way.getNextPathPoint();
    EXPECT_DOUBLE_EQ(p.x, 0.0);
    EXPECT_DOUBLE_EQ(p.y, 0.0);
}

TEST(WayEmpty, QuinEhOnTwoEmptyWaysIsFalse) {
    Way a, b;
    EXPECT_FALSE(a.quinEhLobjetiuDeLaSevaDiresio(b));
}

#include "modules/delaunay_triangulator.hpp"

TEST(DelaunayTriangulatorTest, SpanAndArraySupport) {
    // 验证 std::span 支持固定大小 std::array
    std::array<Node, 4> nodes = {
        Node(0.0, 0.0, 0.0, 0.0, 0),
        Node(2.0, 0.0, 2.0, 0.0, 1),
        Node(1.0, 2.0, 1.0, 2.0, 2),
        Node(1.0, 1.0, 1.0, 1.0, 3)
    };

    // 隐式转换为 std::span<const Node>
    auto triangles = DelaunayTriangulator::compute(nodes);
    EXPECT_FALSE(triangles.empty());
    for (const auto &t : triangles) {
        EXPECT_FALSE(t.anyNodeInSuperTriangle());
    }

    // 验证利用 subspan 进行零拷贝切片剖分
    std::span<const Node> sub_nodes(nodes.data(), 3);
    auto sub_triangles = DelaunayTriangulator::compute(sub_nodes);
    EXPECT_EQ(sub_triangles.size(), 1u);
}

TEST(WayPathTest, GetPathRangesTransform) {
    Way way;
    Node n0(0.0, 0.0, 0.0, 0.0, 0);
    Node n1(2.0, 0.0, 2.0, 0.0, 1);
    Edge e(n0, n1);
    way.addEdge(e);

    auto global_pts = way.getPath();
    ASSERT_EQ(global_pts.size(), 1u);
    EXPECT_DOUBLE_EQ(global_pts[0].x, 1.0);
    EXPECT_DOUBLE_EQ(global_pts[0].y, 0.0);

    auto local_pts = way.getPathLocal();
    ASSERT_EQ(local_pts.size(), 1u);
    EXPECT_DOUBLE_EQ(local_pts[0].x, 1.0);
    EXPECT_DOUBLE_EQ(local_pts[0].y, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
