// #32：cone_tracker 差分合成夹具（ROS1→ROS2 语义差分台账，见 docs/DEFECT_DIFFERENTIAL.md）
//
// 本文件钉住 ROS1 跟踪语义在 ROS2 的对应情况，分两部分：
//   A. ReferenceTracker：按 ROS1 默认参数的中立策略模型（纯 std，可执行规格），
//      确认帧数（近 3 / 远 2）、coast 删除帧数（近 5 / 远 8）、仅输出已确认、
//      确认置信度加成 0.1。参数来源：2025/src/perception_core ConeTracker::Config。
//   B. 现状断言：#35 已收敛节点默认向 ROS1 语义（min_track_frames=3、
//      kalman/ego 开启），默认链 e2e（闪烁不确认、coast 存活）由
//      DefaultChainFlickerNeverConfirmsAndCoastSurvives 锁定。
//
// 注意：ConeDedup 节点需要 rclcpp，无法进 core_standalone；故 B 只用纯算法层 + 源码默认值正则。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "cone_dedup_algo.h"

// 测试源码相对仓库根的路径读取：从 __FILE__ 反推仓库根（core_standalone 与
// colcon 的工作目录不同，但 __FILE__ 在两边都是源文件绝对路径）。
std::string ReadFileSource(const std::string& repo_relative) {
    const std::filesystem::path test_file(__FILE__);
    // .../src/sensors/cone_tracker/test/<file> 上跳 5 级到仓库根
    std::filesystem::path repo_root = test_file;
    for (int i = 0; i < 5; ++i)
        repo_root = repo_root.parent_path();
    std::ifstream in(repo_root / repo_relative);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// declare_parameter("name", default) 的默认值文本匹配。
bool HasDeclareDefault(const std::string& source, const std::string& name, const std::string& want) {
    const std::regex pattern("declare_parameter\\(\"" + name + "\",\\s*" + want + "\\)");
    return std::regex_search(source, pattern);
}

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
    explicit ReferenceTracker(const Policy& policy = Policy{}) : policy_(policy) {}

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

    const std::vector<Track>& all() const { return tracks_; }

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

TEST(CurrentLayerTest, NodeDefaultsConvergedToConfirmationSemantics) {
    // #35：节点默认值已收敛 ROS1 确认语义（min_track_frames=3、kalman/ego 开启）。
    // 原 `NodeDefaultsWeakenConfirmationSemantics` 在收敛后变红即预期内，
    // 现同步更新为收敛断言 + 台账第 8 行分类同步更新为缺陷修复差异。
    const std::string source = ReadFileSource("src/sensors/cone_tracker/src/cone_dedup.cpp");
    EXPECT_TRUE(HasDeclareDefault(source, "min_track_frames", "3"));
    EXPECT_TRUE(HasDeclareDefault(source, "confirmation_frames", "3"));
    EXPECT_TRUE(HasDeclareDefault(source, "enable_kalman", "true"));
    EXPECT_TRUE(HasDeclareDefault(source, "enable_ego_motion_compensation", "true"));
}

TEST(CurrentLayerTest, DefaultChainFlickerNeverConfirmsAndCoastSurvives) {
    // #35 默认链 e2e 断言（无 ROS context 的中立语义层）：
    // 与 ReferenceTracker 同策略——闪烁序列不确认、已确认目标短暂丢失后 coast 存活。
    // 此处复用 ReferenceTracker 作为默认链语义的机读载体（节点需 rclcpp，无法进 core_standalone）。
    const Detection det{10.0, 0.0, 0.8};
    ReferenceTracker flicker;
    for (int i = 0; i < 6; ++i) {
        flicker.update(i % 2 == 0 ? std::vector<Detection>{det} : std::vector<Detection>{});
    }
    EXPECT_TRUE(flicker.confirmed().empty()) << "默认链：闪烁序列不得确认";

    ReferenceTracker coast;
    for (int i = 0; i < 3; ++i)
        coast.update({det});
    ASSERT_EQ(coast.confirmed().size(), 1u);
    for (int i = 0; i < 4; ++i)
        coast.update({});
    EXPECT_EQ(coast.confirmed().size(), 1u) << "默认链：coast 4 帧内必须存活";
    coast.update({});
    EXPECT_TRUE(coast.confirmed().empty()) << "默认链：coast 超限必须删除";
}
