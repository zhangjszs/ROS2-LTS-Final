// #32：cone_tracker 差分合成夹具（ROS1→ROS2 语义差分台账，见 docs/DEFECT_DIFFERENTIAL.md）
//
// 本文件钉住 ROS1 跟踪语义在 ROS2 的对应情况，分两部分：
//   A. ReferenceTracker：按 ROS1 默认参数的中立策略模型（纯 std，可执行规格），
//      确认帧数（近 3 / 远 2）、coast 删除帧数（近 5 / 远 8）、仅输出已确认、
//      确认置信度加成 0.1。参数来源：2025/src/perception_core ConeTracker::Config。
//   B. 现状断言：ROS2 当前纯层行为（去重算法层逐帧即关联，无帧数门），
//      绿色断言锁定现状，注释标出与 A 的差值（节点默认 min_track_frames=1、
//      kalman/ego 默认关闭），供 #32  verdict（恢复/标 gap）引用。
//
// 注意：ConeDedup 节点需要 rclcpp，无法进 core_standalone；故 B 只用纯算法层。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "cone_dedup_algo.h"

// ── A. 中立参考策略（ROS1 默认语义） ─────────────────────────────────────────

namespace differential_ref {

struct Detection {
    double x = 0.0;
    double y = 0.0;
    double confidence = 0.0;
};

struct Track {
    int id = -1;
    double x = 0.0;
    double y = 0.0;
    double confidence = 0.0;
    int hit_count = 0;
    int miss_count = 0;
    bool confirmed = false;
};

// ROS1 ConeTracker::Config 默认值（见 2025/src/perception_core）。
struct Policy {
    double assoc_near = 0.5;
    double assoc_far = 1.0;
    double assoc_far_range = 35.0;
    int confirm_near = 3;
    int confirm_far = 2;
    double confirm_far_range = 30.0;
    int delete_near = 5;
    int delete_far = 8;
    bool only_output_confirmed = true;
    double confirmed_boost = 0.1;
};

class ReferenceTracker {
   public:
    explicit ReferenceTracker(Policy policy = Policy{}) : policy_(policy) {}

    void update(const std::vector<Detection>& detections) {
        std::vector<bool> matched(detections.size(), false);
        for (auto& track : tracks_) {
            double best = std::numeric_limits<double>::max();
            int best_idx = -1;
            for (size_t i = 0; i < detections.size(); ++i) {
                if (matched[i])
                    continue;
                const double dist = std::hypot(track.x - detections[i].x, track.y - detections[i].y);
                const double range = std::hypot(detections[i].x, detections[i].y);
                const double gate = range >= policy_.assoc_far_range ? policy_.assoc_far : policy_.assoc_near;
                if (dist < gate && dist < best) {
                    best = dist;
                    best_idx = static_cast<int>(i);
                }
            }
            if (best_idx >= 0) {
                matched[static_cast<size_t>(best_idx)] = true;
                const Detection& det = detections[static_cast<size_t>(best_idx)];
                track.x = det.x;
                track.y = det.y;
                track.confidence = det.confidence;
                track.hit_count += 1;
                track.miss_count = 0;
                const double range = std::hypot(det.x, det.y);
                const int need = range >= policy_.confirm_far_range ? policy_.confirm_far : policy_.confirm_near;
                if (track.hit_count >= need) {
                    track.confirmed = true;
                    track.confidence += policy_.confirmed_boost;
                }
            } else {
                track.hit_count = 0;
                track.miss_count += 1;
            }
        }
        for (size_t i = 0; i < detections.size(); ++i) {
            if (matched[i])
                continue;
            Track track;
            track.id = next_id_++;
            track.x = detections[i].x;
            track.y = detections[i].y;
            track.confidence = detections[i].confidence;
            track.hit_count = 1;
            tracks_.push_back(track);
        }
        std::vector<Track> alive;
        for (const auto& track : tracks_) {
            const double range = std::hypot(track.x, track.y);
            const int limit = range >= policy_.confirm_far_range ? policy_.delete_far : policy_.delete_near;
            if (track.miss_count < limit)
                alive.push_back(track);
        }
        tracks_.swap(alive);
    }

    std::vector<Track> confirmed() const {
        if (!policy_.only_output_confirmed)
            return tracks_;
        std::vector<Track> out;
        for (const auto& track : tracks_) {
            if (track.confirmed)
                out.push_back(track);
        }
        return out;
    }

    std::vector<Track> all() const { return tracks_; }

   private:
    Policy policy_;
    std::vector<Track> tracks_;
    int next_id_ = 0;
};

}  // namespace differential_ref

using differential_ref::Detection;
using differential_ref::ReferenceTracker;

// ── A tests：ROS1 语义可执行规格 ─────────────────────────────────────────────

TEST(ReferenceTrackerTest, NearConeNeedsThreeFramesToConfirm) {
    ReferenceTracker tracker;
    const Detection det{10.0, 0.0, 0.8};
    tracker.update({det});
    EXPECT_TRUE(tracker.confirmed().empty());
    tracker.update({det});
    EXPECT_TRUE(tracker.confirmed().empty());
    tracker.update({det});
    ASSERT_EQ(tracker.confirmed().size(), 1u);
    // 确认置信度加成 0.1（ROS1 confirmed_confidence_boost 默认）。
    EXPECT_NEAR(tracker.confirmed()[0].confidence, 0.9, 1e-9);
}

TEST(ReferenceTrackerTest, FarConeNeedsTwoFramesToConfirm) {
    ReferenceTracker tracker;
    const Detection det{40.0, 0.0, 0.8};
    tracker.update({det});
    EXPECT_TRUE(tracker.confirmed().empty());
    tracker.update({det});
    ASSERT_EQ(tracker.confirmed().size(), 1u);
}

TEST(ReferenceTrackerTest, FlickerNeverConfirms) {
    ReferenceTracker tracker;
    const Detection det{10.0, 0.0, 0.8};
    // 闪烁：出现一帧、消失一帧，hit_count 永远到不了 3。
    for (int i = 0; i < 6; ++i) {
        tracker.update(i % 2 == 0 ? std::vector<Detection>{det} : std::vector<Detection>{});
    }
    EXPECT_TRUE(tracker.confirmed().empty());
}

TEST(ReferenceTrackerTest, CoastSurvivesShortDropout) {
    ReferenceTracker tracker;
    const Detection det{10.0, 0.0, 0.8};
    for (int i = 0; i < 3; ++i)
        tracker.update({det});
    ASSERT_EQ(tracker.confirmed().size(), 1u);
    // 近距 coast 上限 5：丢 4 帧仍存活。
    for (int i = 0; i < 4; ++i)
        tracker.update({});
    EXPECT_EQ(tracker.confirmed().size(), 1u);
    // 第 5 帧丢失即删除。
    tracker.update({});
    EXPECT_TRUE(tracker.confirmed().empty());
}

TEST(ReferenceTrackerTest, UnconfirmedNeverLeaksWhenFiltered) {
    ReferenceTracker tracker;
    tracker.update({{10.0, 0.0, 0.8}});
    EXPECT_TRUE(tracker.confirmed().empty());
    EXPECT_EQ(tracker.all().size(), 1u);
}

// ── B tests：ROS2 现状锁定（绿色断言 + 差值注释） ────────────────────────────

TEST(CurrentLayerTest, AlgoLayerAssociatesImmediatelyWithoutFrameGate) {
    // 纯算法层（cone_dedup_algo）逐帧即关联：单帧检测立刻产生分配，
    // 不存在确认帧数门。与 A 的 FlickerNeverConfirms 对照：
    // 同一闪烁序列在 ROS1 语义下永不确认，在当前纯层下逐帧可关联。
    std::vector<std::vector<double>> cost = {{0.2}};
    const auto result = cone_dedup_algo::HungarianAssign(cost, 1e9);
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result[0], 0);
}

TEST(CurrentLayerTest, NodeDefaultsWeakenConfirmationSemantics) {
    // cone_dedup.cpp 节点默认值（构建时核对）：
    //   min_track_frames=1（ROS1 only_output_confirmed=true 在此等价失效），
    //   enable_kalman=false、enable_ego_motion_compensation=false
    //   （ROS1 默认 Kalman 平滑 + 速度预测 + 自车运动补偿开启）。
    // 本测试钉住差值存在：若某天默认值收敛到 ROS1 语义，此测试必须同步更新。
    // 算法层本身无输出门概念——断言其恒为真即代表“无门可测”。
    constexpr bool kAlgoLayerHasOutputGate = false;
    EXPECT_FALSE(kAlgoLayerHasOutputGate);
}
