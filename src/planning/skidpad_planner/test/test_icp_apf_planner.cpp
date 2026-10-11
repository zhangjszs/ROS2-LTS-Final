#include <gtest/gtest.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>
#include <ranges>
#include <vector>

#include "icp_apf_planner.hpp"

namespace {

void LegacyAppendMidpointsPCL(const std::vector<skidpad::Point2D>& primary,
                              const std::vector<skidpad::Point2D>& secondary, double max_dist,
                              std::vector<skidpad::Point2D>& out) {
    if (primary.empty() || secondary.empty()) {
        return;
    }

    const double max_dist_sq = max_dist * max_dist;

    pcl::PointCloud<pcl::PointXY>::Ptr sec_cloud(new pcl::PointCloud<pcl::PointXY>);
    sec_cloud->points.reserve(secondary.size());
    for (const auto& s : secondary) {
        pcl::PointXY pt;
        pt.x = static_cast<float>(s.x);
        pt.y = static_cast<float>(s.y);
        sec_cloud->points.push_back(pt);
    }
    sec_cloud->width = sec_cloud->points.size();
    sec_cloud->height = 1;
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(sec_cloud);

    for (const auto& p : primary) {
        const double px = p.x;
        const double py = p.y;
        pcl::PointXY query;
        query.x = static_cast<float>(px);
        query.y = static_cast<float>(py);
        std::vector<int> idx(1);
        std::vector<float> dist_sq(1);
        if (kdtree.nearestKSearch(query, 1, idx, dist_sq) == 1 && dist_sq[0] <= max_dist_sq) {
            const auto& best = secondary[static_cast<size_t>(idx[0])];
            out.emplace_back((px + best.x) * 0.5, (py + best.y) * 0.5);
        }
    }
}

std::vector<skidpad::Point2D> LegacyComputeCenterlinePCL(const std::vector<skidpad::Point2D>& left,
                                                         const std::vector<skidpad::Point2D>& right,
                                                         const skidpad::CenterlineParams& p) {
    if (left.empty() || right.empty()) {
        return {};
    }

    std::vector<skidpad::Point2D> centerline;
    centerline.reserve(left.size() + right.size());

    LegacyAppendMidpointsPCL(left, right, p.max_pair_dist, centerline);
    LegacyAppendMidpointsPCL(right, left, p.max_pair_dist, centerline);

    if (centerline.empty()) {
        return {};
    }

    std::vector<skidpad::Point2D> ordered;
    ordered.reserve(centerline.size());
    std::vector<bool> used(centerline.size(), false);
    auto take_nearest = [&](double x0, double y0) -> int {
        auto unused_indices =
            std::views::iota(size_t{0}, centerline.size()) | std::views::filter([&used](size_t i) { return !used[i]; });
        auto it = std::ranges::min_element(unused_indices, {}, [&](size_t i) {
            const double dx = centerline[i].x - x0;
            const double dy = centerline[i].y - y0;
            return (dx * dx) + (dy * dy);
        });
        if (it == unused_indices.end()) {
            return -1;
        }
        return static_cast<int>(*it);
    };
    int start = take_nearest(0.0, 0.0);
    while (start >= 0) {
        used[static_cast<size_t>(start)] = true;
        ordered.push_back(centerline[static_cast<size_t>(start)]);
        start = take_nearest(ordered.back().x, ordered.back().y);
    }
    centerline.swap(ordered);

    const double dedup_sq = (p.path_spacing * 0.5) * (p.path_spacing * 0.5);
    std::vector<skidpad::Point2D> deduped;
    deduped.reserve(centerline.size());
    for (const auto& pt : centerline) {
        if (deduped.empty()) {
            deduped.push_back(pt);
            continue;
        }
        const double dx = pt.x - deduped.back().x;
        const double dy = pt.y - deduped.back().y;
        if ((dx * dx) + (dy * dy) > dedup_sq) {
            deduped.push_back(pt);
        }
    }

    std::vector<skidpad::Point2D> result;
    result.reserve(deduped.size());
    const double lookahead_sq = p.path_lookahead * p.path_lookahead;
    std::ranges::copy_if(
        deduped, std::back_inserter(result), [lookahead_sq](double d2) { return d2 <= lookahead_sq; },
        [](const skidpad::Point2D& pt) { return (pt.x * pt.x) + (pt.y * pt.y); });
    return result;
}

}  // namespace

static common_msgs::msg::HuatCone MakeCone(double x, double y) {
    common_msgs::msg::HuatCone c;
    c.position_base_link.x = x;
    c.position_base_link.y = y;
    return c;
}

TEST(IcpApfPlanner, EmptyConesReturnsPreviousPath) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<skidpad::Point2D> prev = {skidpad::Point2D(1.0, 0.0), skidpad::Point2D(2.0, 0.0)};
    auto path = planner.GeneratePath({}, prev);
    ASSERT_EQ(path.size(), 2u);
    EXPECT_NEAR(path[0].x, 1.0, 1e-12);
    EXPECT_NEAR(path[1].x, 2.0, 1e-12);
}

TEST(IcpApfPlanner, OneSidedConesReturnsPreviousPath) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 0; i < 6; ++i)
        cones.push_back(MakeCone(static_cast<double>(i), 1.5));
    auto path = planner.GeneratePath(cones, {});
    EXPECT_TRUE(path.empty());
}

TEST(IcpApfPlanner, OppositeSidesProduceForwardPath) {
    skidpad::IcpApfConfig cfg;
    cfg.path_lookahead = 20.0;
    cfg.path_spacing = 0.5;
    skidpad::IcpApfPlanner planner(cfg);
    std::vector<common_msgs::msg::HuatCone> cones;
    for (int i = 1; i <= 8; ++i) {
        cones.push_back(MakeCone(static_cast<double>(i), -1.5));
        cones.push_back(MakeCone(static_cast<double>(i), 1.5));
    }
    auto path = planner.GeneratePath(cones, {});
    ASSERT_FALSE(path.empty());
    EXPECT_GT(path.front().x, 0.0);
}

TEST(IcpApfPlanner, ClusterConesRangesPartitioning) {
    skidpad::IcpApfConfig cfg;
    cfg.center_margin = 0.3;
    skidpad::IcpApfPlanner planner(cfg);

    std::vector<common_msgs::msg::HuatCone> cones = {
        MakeCone(1.0, -1.5),  // left
        MakeCone(2.0, 1.5),   // right
        MakeCone(3.0, -0.2),  // within margin, should be skipped
        MakeCone(4.0, 0.1),   // within margin, should be skipped
        MakeCone(5.0, -2.0),  // left
        MakeCone(6.0, 2.5)    // right
    };

    std::vector<common_msgs::msg::HuatCone> left, right;
    planner.ClusterCones(cones, left, right);

    EXPECT_EQ(left.size(), 2u);
    EXPECT_EQ(right.size(), 2u);
    EXPECT_DOUBLE_EQ(left[0].position_base_link.y, -1.5);
    EXPECT_DOUBLE_EQ(left[1].position_base_link.y, -2.0);
    EXPECT_DOUBLE_EQ(right[0].position_base_link.y, 1.5);
    EXPECT_DOUBLE_EQ(right[1].position_base_link.y, 2.5);
}

TEST(IcpApfPlanner, TransformPointsRanges) {
    skidpad::IcpApfConfig cfg;
    skidpad::IcpApfPlanner planner(cfg);

    std::vector<skidpad::Point2D> pts = {{1.0, 0.0}, {0.0, 1.0}};

    // Rotate 90 deg counter-clockwise, translate dx=2.0, dy=3.0
    double rot = std::numbers::pi_v<double> / 2.0;
    auto transformed = planner.TransformPoints(pts, rot, 2.0, 3.0);

    ASSERT_EQ(transformed.size(), 2u);
    EXPECT_NEAR(transformed[0].x, 2.0, 1e-6);
    EXPECT_NEAR(transformed[0].y, 4.0, 1e-6);
    EXPECT_NEAR(transformed[1].x, 1.0, 1e-6);
    EXPECT_NEAR(transformed[1].y, 3.0, 1e-6);
}

TEST(IcpApfPlanner, DifferentialEquivalenceWithLegacyPCL) {
    // 1000 trials with fixed seed (reproducible)
    constexpr uint64_t kSeed = 20261011ULL;
    std::mt19937_64 rng(kSeed);
    std::uniform_int_distribution<size_t> num_cones_dist(4, 16);
    std::uniform_real_distribution<double> x_dist(0.5, 18.0);
    std::uniform_real_distribution<double> y_left_dist(-4.0, -1.0);
    std::uniform_real_distribution<double> y_right_dist(1.0, 4.0);

    const skidpad::CenterlineParams params{
        .max_pair_dist = 6.0,
        .path_spacing = 0.5,
        .path_lookahead = 20.0,
    };

    constexpr size_t kNumTrials = 1000;
    for (size_t trial = 0; trial < kNumTrials; ++trial) {
        const size_t n_left = num_cones_dist(rng);
        const size_t n_right = num_cones_dist(rng);

        std::vector<skidpad::Point2D> left;
        left.reserve(n_left);
        for (size_t i = 0; i < n_left; ++i) {
            left.emplace_back(x_dist(rng), y_left_dist(rng));
        }

        std::vector<skidpad::Point2D> right;
        right.reserve(n_right);
        for (size_t i = 0; i < n_right; ++i) {
            right.emplace_back(x_dist(rng), y_right_dist(rng));
        }

        auto legacy_out = LegacyComputeCenterlinePCL(left, right, params);
        auto core_out = skidpad::ComputeCenterlineCore(left, right, params);

        ASSERT_EQ(core_out.size(), legacy_out.size()) << "Mismatch in output size on trial " << trial;

        for (size_t i = 0; i < core_out.size(); ++i) {
            EXPECT_TRUE(core_out[i].x == legacy_out[i].x)
                << "Trial " << trial << " pt " << i << " x mismatch: " << core_out[i].x << " vs " << legacy_out[i].x;
            EXPECT_TRUE(core_out[i].y == legacy_out[i].y)
                << "Trial " << trial << " pt " << i << " y mismatch: " << core_out[i].y << " vs " << legacy_out[i].y;
        }
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
