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
