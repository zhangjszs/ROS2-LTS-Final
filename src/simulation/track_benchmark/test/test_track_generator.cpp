// #81：赛道终点/缓冲区元数据（finish_x / runout_end_m）——生成器、manifest 与几何的一致性。
// 消费方（离线 runner 的完赛判定、ROS 闭环门禁）必须能只靠元数据拿到 75/100，
// 不得再硬编码，也不得从 RED 锥桶反推终点位置。
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "track_benchmark/track_generator.hpp"

using namespace benchmark;

namespace {

// 导出 manifest 到临时目录并读回全文（不落仓内，避免污染工作区）。
std::string ExportManifest(const std::vector<TrackDefinition>& tracks, const std::vector<std::string>& files) {
    const auto dir = std::filesystem::temp_directory_path() / "fsac_track_generator_manifest";
    std::filesystem::create_directories(dir);
    if (!TrackGenerator::SaveTrackManifest(tracks, dir.string(), files))
        return {};
    std::ifstream f(dir / "tracks.json");
    std::ostringstream os;
    os << f.rdbuf();
    return os.str();
}

// manifest 里每个赛道占一行；按 id 取出该行，便于逐字段断言。
std::string ManifestLine(const std::string& manifest, const std::string& id) {
    std::istringstream is(manifest);
    std::string line;
    while (std::getline(is, line)) {
        if (line.find("\"id\": \"" + id + "\"") != std::string::npos)
            return line;
    }
    return {};
}

}  // namespace

// —— 生成器：终点元数据随几何一同产生 ——

TEST(TrackFinishMetadataTest, AccelerationCarriesFinishLineAndRunout) {
    const auto accel = TrackGenerator::GenerateAcceleration();
    ASSERT_TRUE(accel.finish_x.has_value());
    ASSERT_TRUE(accel.runout_end_m.has_value());
    EXPECT_DOUBLE_EQ(*accel.finish_x, 75.0);
    EXPECT_DOUBLE_EQ(*accel.runout_end_m, 100.0);
    EXPECT_TRUE(accel.hasFinish());
}

TEST(TrackFinishMetadataTest, FinishMetadataFollowsLengthParameter) {
    // 元数据不得与参数脱钩：改长度时终点线与缓冲区末端同步跟随。
    const auto accel = TrackGenerator::GenerateAcceleration(50.0);
    ASSERT_TRUE(accel.finish_x.has_value());
    ASSERT_TRUE(accel.runout_end_m.has_value());
    EXPECT_DOUBLE_EQ(*accel.finish_x, 50.0);
    EXPECT_DOUBLE_EQ(*accel.runout_end_m, 75.0);
}

TEST(TrackFinishMetadataTest, ClosedTracksHaveNoFinishSemantics) {
    for (const auto& t : {TrackGenerator::GenerateSkidpad(), TrackGenerator::GenerateTrackdriveLoop()}) {
        EXPECT_FALSE(t.finish_x.has_value()) << t.id;
        EXPECT_FALSE(t.runout_end_m.has_value()) << t.id;
        EXPECT_FALSE(t.hasFinish()) << t.id;
    }
}

// 元数据必须与实际几何一致（不是凭空写死的数字）：终点线落在 RED 锥桶处，
// 缓冲区末端落在中心线/锥桶的延伸末端。
TEST(TrackFinishMetadataTest, FinishMetadataIsConsistentWithGeometry) {
    const auto accel = TrackGenerator::GenerateAcceleration();
    ASSERT_TRUE(accel.finish_x.has_value());
    ASSERT_TRUE(accel.runout_end_m.has_value());
    // 中心线延伸到缓冲区末端。
    EXPECT_NEAR(accel.centerline.back().x, *accel.runout_end_m, 1e-9);
    // 终点线处有 RED 锥桶（赛制语义与几何同源）。
    bool red_at_finish = false;
    for (const auto& c : accel.cones) {
        if (c.type == huat_cone::RED && std::abs(c.x - *accel.finish_x) < 1e-9)
            red_at_finish = true;
    }
    EXPECT_TRUE(red_at_finish);
    // 缓冲区末端之后不再有锥桶（终点后是空缓冲，不该再摆锥）。
    for (const auto& c : accel.cones) {
        EXPECT_LE(c.x, *accel.runout_end_m + 1e-9);
    }
}

// —— 几何不变：本单只加元数据，不动几何 ——

TEST(TrackFinishMetadataTest, GeometryChecksumAndLengthUnchanged) {
    const auto accel = TrackGenerator::GenerateAcceleration();
    // 与已提交 CSV 头里的 geometry_fnv1a64 一致（几何未漂移 → 不需要递增 track_version）。
    EXPECT_EQ(TrackGenerator::geometryChecksum(accel.cones), 0x64291c72658ad63fULL);
    EXPECT_EQ(accel.cones.size(), 76u);
    // 中心线从 x=-5 到 x=100 共 211 点、步长 0.5m，累加器多走一步 → 105.5m（与旧行为一致）
    EXPECT_DOUBLE_EQ(accel.total_length, 105.5);
}

// —— manifest：机读消费方拿得到 75/100 ——

TEST(TrackFinishMetadataTest, ManifestWritesFinishMetadataForAcceleration) {
    const std::vector<TrackDefinition> tracks = {TrackGenerator::GenerateAcceleration(),
                                                 TrackGenerator::GenerateSkidpad(),
                                                 TrackGenerator::GenerateTrackdriveLoop()};
    const std::vector<std::string> files = {"acceleration_track.csv", "skidpad_track.csv", "trackdrive_loop.csv"};
    const std::string manifest = ExportManifest(tracks, files);
    ASSERT_FALSE(manifest.empty()) << "manifest 未写出";
    EXPECT_NE(manifest.find("\"schema\": \"fsac.tracks/v1\""), std::string::npos);

    const std::string accel = ManifestLine(manifest, "acceleration-75m");
    ASSERT_FALSE(accel.empty()) << "manifest 中找不到 acceleration-75m 行";
    EXPECT_NE(accel.find("\"finish_x\": 75.000"), std::string::npos) << accel;
    EXPECT_NE(accel.find("\"runout_end_m\": 100.000"), std::string::npos) << accel;
}

TEST(TrackFinishMetadataTest, ManifestWritesNullFinishMetadataForClosedTracks) {
    const std::vector<TrackDefinition> tracks = {TrackGenerator::GenerateAcceleration(),
                                                 TrackGenerator::GenerateSkidpad(),
                                                 TrackGenerator::GenerateTrackdriveLoop()};
    const std::vector<std::string> files = {"acceleration_track.csv", "skidpad_track.csv", "trackdrive_loop.csv"};
    const std::string manifest = ExportManifest(tracks, files);
    ASSERT_FALSE(manifest.empty());

    for (const char* id : {"skidpad-figure8", "trackdrive-loop"}) {
        const std::string line = ManifestLine(manifest, id);
        ASSERT_FALSE(line.empty()) << id;
        EXPECT_NE(line.find("\"finish_x\": null"), std::string::npos) << line;
        EXPECT_NE(line.find("\"runout_end_m\": null"), std::string::npos) << line;
    }
}

// —— #88：八字绕环完整中心线几何断言（包含左右双圆、正负曲率极值、各段连续性）——
TEST(TrackSkidpadGeometryTest, SkidpadCenterlineCoversBothCirclesWithCurvature) {
    const auto skidpad = TrackGenerator::GenerateSkidpad();
    ASSERT_FALSE(skidpad.centerline.empty());

    // 1. 验证存在覆盖左右两圆的极值点 (y > 5.0 与 y < -5.0)
    bool has_right_extreme = false;
    bool has_left_extreme = false;
    double max_y = -1e9;
    double min_y = 1e9;

    // 2. 验证曲率包含正负两种极值 (右圆 +1/9.125, 左圆 -1/9.125)
    bool has_pos_curvature = false;
    bool has_neg_curvature = false;
    const double expected_pos_kappa = 1.0 / 9.125;
    const double expected_neg_kappa = -1.0 / 9.125;

    // 3. 验证各段弧长累积连续不降
    double prev_s = 0.0;
    for (size_t i = 0; i < skidpad.centerline.size(); ++i) {
        const auto& pt = skidpad.centerline[i];
        max_y = std::max(max_y, pt.y);
        min_y = std::min(min_y, pt.y);
        if (pt.y > 5.0)
            has_right_extreme = true;
        if (pt.y < -5.0)
            has_left_extreme = true;

        if (std::abs(pt.curvature - expected_pos_kappa) < 1e-4)
            has_pos_curvature = true;
        if (std::abs(pt.curvature - expected_neg_kappa) < 1e-4)
            has_neg_curvature = true;

        if (i > 0) {
            EXPECT_GE(pt.s, prev_s) << "Arc length decreased at point " << i;
            EXPECT_LE(pt.s - prev_s, 1.0) << "Arc length step too large at point " << i;
        }
        prev_s = pt.s;
    }

    EXPECT_TRUE(has_right_extreme);
    EXPECT_TRUE(has_left_extreme);
    EXPECT_GT(max_y, 18.0);   // cy_r(9.125) + r_mid(9.125) = 18.25
    EXPECT_LT(min_y, -18.0);  // cy_l(-9.125) - r_mid(9.125) = -18.25

    EXPECT_TRUE(has_pos_curvature);
    EXPECT_TRUE(has_neg_curvature);

    // 4. 总里程与起点终点断言
    EXPECT_GT(skidpad.total_length, 250.0);
    EXPECT_NEAR(skidpad.centerline.front().x, -15.0, 1e-6);
    EXPECT_NEAR(skidpad.centerline.front().y, 0.0, 1e-6);
    EXPECT_NEAR(skidpad.centerline.back().x, 15.0, 1e-6);
    EXPECT_NEAR(skidpad.centerline.back().y, 0.0, 1e-6);
}
