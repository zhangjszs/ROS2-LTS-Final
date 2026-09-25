#include "track_benchmark/track_generator.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <sstream>

namespace benchmark {

namespace {

constexpr double kPi = std::numbers::pi_v<double>;

}  // namespace

TrackDefinition TrackGenerator::GenerateSkidpad() {
    TrackDefinition track;
    track.name = "Skidpad";
    track.id = "skidpad-figure8";  // #17：全仓唯一赛道标识
    track.track_width = 3.0;

    // FSAC 规则: 右圆中心 (0, 9.125), 左圆中心 (0, -9.125)
    // 内圆半径 7.625m, 外圆半径 10.625m, 赛道中心半径 R_mid = 9.125m
    const double r_inner = 7.625;
    const double r_outer = 10.625;
    const double r_mid = 9.125;
    uint32_t cone_id = 1;

    // 1. 直线进出场通道 (从 x = -15m 到 x = 0m, 宽 3m)
    for (double x = -15.0; x <= 0.0; x += 3.0) {
        track.cones.push_back({.x = x, .y = 1.5, .type = huat_cone::BLUE, .id = cone_id++});
        track.cones.push_back({.x = x, .y = -1.5, .type = huat_cone::YELLOW, .id = cone_id++});
    }

    // 2. 右圆 (y > 0): 逆时针
    const double cy_r = 9.125;
    for (int deg = 0; deg < 360; deg += 15) {
        double rad = deg * (kPi / 180.0);
        track.cones.push_back({.x = r_inner * std::cos(rad),
                               .y = cy_r + r_inner * std::sin(rad),
                               .type = huat_cone::YELLOW,
                               .id = cone_id++});
        track.cones.push_back({.x = r_outer * std::cos(rad),
                               .y = cy_r + r_outer * std::sin(rad),
                               .type = huat_cone::BLUE,
                               .id = cone_id++});
    }

    // 3. 左圆 (y < 0): 顺时针
    const double cy_l = -9.125;
    for (int deg = 0; deg < 360; deg += 15) {
        double rad = deg * (kPi / 180.0);
        track.cones.push_back({.x = r_inner * std::cos(rad),
                               .y = cy_l + r_inner * std::sin(rad),
                               .type = huat_cone::BLUE,
                               .id = cone_id++});
        track.cones.push_back({.x = r_outer * std::cos(rad),
                               .y = cy_l + r_outer * std::sin(rad),
                               .type = huat_cone::YELLOW,
                               .id = cone_id++});
    }

    // 4. 中心参考航路点: 进场直线 + 右圆双圈 + 左圆双圈
    double s = 0.0;
    // 进场线
    for (double x = -15.0; x < 0.0; x += 0.5) {
        track.centerline.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .s = s});
        s += 0.5;
    }
    // 右圆参考轨迹 (中心 R_mid = 9.125m, 曲率 kappa = 1 / 9.125)
    const double d_theta = (kPi / 180.0) * 5.0;  // 每 5 度一个参考点
    for (double th = -kPi / 2.0; th <= 3.0 * kPi / 2.0; th += d_theta) {
        double px = r_mid * std::cos(th);
        double py = cy_r + r_mid * std::sin(th);
        double heading = th + kPi / 2.0;
        track.centerline.push_back({.x = px, .y = py, .theta = heading, .curvature = 1.0 / r_mid, .s = s});
        s += r_mid * d_theta;
    }

    track.total_length = s;
    return track;
}

TrackDefinition TrackGenerator::GenerateAcceleration(double length, double track_width) {
    TrackDefinition track;
    track.name = "Acceleration";
    track.id = "acceleration-75m";
    track.closed_circuit = false;  // 直线赛：到终点即完赛，不计圈
    track.track_width = track_width;
    const double half_w = track_width * 0.5;
    uint32_t cone_id = 1;

    // 赛道两侧锥桶 (从 -5m 到 length + 25m)
    for (double x = -5.0; x <= length + 25.0; x += 3.0) {
        track.cones.push_back({.x = x, .y = half_w, .type = huat_cone::BLUE, .id = cone_id++});
        track.cones.push_back({.x = x, .y = -half_w, .type = huat_cone::YELLOW, .id = cone_id++});
    }

    // 起跑线大橙色锥桶
    track.cones.push_back({.x = 0.0, .y = half_w, .type = huat_cone::YELLOW_BIG, .id = cone_id++});
    track.cones.push_back({.x = 0.0, .y = -half_w, .type = huat_cone::YELLOW_BIG, .id = cone_id++});

    // 75m 终点线红色锥桶
    track.cones.push_back({.x = length, .y = half_w, .type = huat_cone::RED, .id = cone_id++});
    track.cones.push_back({.x = length, .y = -half_w, .type = huat_cone::RED, .id = cone_id++});

    // 中心参考航路点
    double s = 0.0;
    for (double x = -5.0; x <= length + 25.0; x += 0.5) {
        track.centerline.push_back({.x = x, .y = 0.0, .theta = 0.0, .curvature = 0.0, .s = s});
        s += 0.5;
    }
    track.total_length = s;
    return track;
}

TrackDefinition TrackGenerator::GenerateTrackdriveLoop(double rx, double ry, double track_width, size_t num_points) {
    TrackDefinition track;
    track.name = "Trackdrive_Loop";
    track.id = "trackdrive-loop";
    track.track_width = track_width;
    const double half_w = track_width * 0.5;
    uint32_t cone_id = 1;

    double s = 0.0;
    std::vector<CenterlinePoint> raw_centerline;
    raw_centerline.reserve(num_points);

    for (size_t i = 0; i < num_points; ++i) {
        double t = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(num_points);
        // 参数化中心线: 包含主椭圆与连续 S 弯复合线
        double cx = rx * std::sin(t);
        double cy = ry * std::cos(t) + 4.0 * std::sin(2.0 * t);

        // 一阶导数 (切线向量)
        double dx = rx * std::cos(t);
        double dy = -ry * std::sin(t) + 8.0 * std::cos(2.0 * t);
        double norm = std::hypot(dx, dy);
        double heading = std::atan2(dy, dx);

        // 二阶导数 (用于计算真实曲率 kappa)
        double ddx = -rx * std::sin(t);
        double ddy = -ry * std::cos(t) - 16.0 * std::sin(2.0 * t);
        double curvature = (dx * ddy - dy * ddx) / std::pow(norm, 3.0);

        // 法线向量 (垂直切线向左)
        double nx = -dy / norm;
        double ny = dx / norm;

        // 左右边界锥桶 (每 2 个点插一排锥桶以防过于密集)
        if (i % 2 == 0) {
            track.cones.push_back(
                {.x = cx + nx * half_w, .y = cy + ny * half_w, .type = huat_cone::BLUE, .id = cone_id++});
            track.cones.push_back(
                {.x = cx - nx * half_w, .y = cy - ny * half_w, .type = huat_cone::YELLOW, .id = cone_id++});
        }

        if (i > 0) {
            double ds = std::hypot(cx - raw_centerline.back().x, cy - raw_centerline.back().y);
            s += ds;
        }

        raw_centerline.push_back({.x = cx, .y = cy, .theta = heading, .curvature = curvature, .s = s});
    }

    track.centerline = std::move(raw_centerline);
    track.total_length = s;
    return track;
}

bool TrackGenerator::SaveConesToCSV(const TrackDefinition& track, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open())
        return false;
    // 固定 3 位小数 + 确定行序：输出位级确定，才能与仓内已提交的 CSV 逐字节 diff（#17 单一赛道来源）。
    // 身份写在注释行里（两个读取方都跳过 '#' 行），使每份几何自带 id/version/校验和。
    f << "# x,y,type\n";
    f << "# id=" << track.id << " version=" << track.version << " cones=" << track.cones.size()
      << " geometry_fnv1a64=" << std::hex << geometryChecksum(track.cones) << std::dec << "\n";
    f << std::fixed << std::setprecision(3);
    for (const auto& c : track.cones) {
        f << c.x << "," << c.y << "," << c.type << "\n";
    }
    return true;
}

std::uint64_t TrackGenerator::geometryChecksum(const std::vector<BenchmarkCone>& cones) noexcept {
    // 对“量化到导出精度后的字节流”取 FNV-1a：校验和衡量的是导出几何，不是内存里的 double 尾数。
    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    for (const auto& c : cones)
        os << c.x << "," << c.y << "," << c.type << "\n";
    const std::string bytes = os.str();
    std::uint64_t h = 1469598103934665603ULL;  // FNV-1a 64bit offset basis
    for (unsigned char b : bytes) {
        h ^= b;
        h *= 1099511628211ULL;
    }
    return h;
}

bool TrackGenerator::SaveTrackManifest(const std::vector<TrackDefinition>& tracks, const std::string& dir,
                                       const std::vector<std::string>& filenames) {
    if (tracks.size() != filenames.size())
        return false;
    std::ofstream f(dir + "/tracks.json");
    if (!f.is_open())
        return false;
    // schema fsac.tracks/v1：仿真器与评测共用同一份清单（id/version/文件名/锥桶数/几何校验和）。
    f << "{\n  \"schema\": \"fsac.tracks/v1\",\n  \"tracks\": [\n";
    for (size_t i = 0; i < tracks.size(); ++i) {
        const auto& t = tracks[i];
        // 校验和先转成十六进制字符串再拼接：在流里直接切 std::hex/std::dec 容易把后续字段带成十六进制。
        std::ostringstream sum;
        sum << std::hex << geometryChecksum(t.cones);
        f << "    {\"id\": \"" << t.id << "\", \"version\": \"" << t.version << "\", \"name\": \"" << t.name
          << "\", \"file\": \"" << filenames[i] << "\", \"cones\": " << t.cones.size()
          << ", \"closed\": " << (t.closed_circuit ? "true" : "false") << ", \"total_length_m\": " << std::fixed
          << std::setprecision(3) << t.total_length << ", \"geometry_fnv1a64\": \"" << sum.str() << "\"}"
          << (i + 1 == tracks.size() ? "" : ",") << "\n";
    }
    f << "  ]\n}\n";
    return f.good();
}

bool TrackGenerator::SaveCenterlineToCSV(const std::vector<CenterlinePoint>& centerline, const std::string& path) {
    std::ofstream f(path);
    if (!f.is_open())
        return false;
    f << "# x,y,theta,curvature,s\n";
    for (const auto& pt : centerline) {
        f << pt.x << "," << pt.y << "," << pt.theta << "," << pt.curvature << "," << pt.s << "\n";
    }
    return true;
}

std::vector<CenterlinePoint> TrackGenerator::LoadCenterlineFromCSV(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open())
        return {};
    std::vector<CenterlinePoint> pts;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::stringstream ss(line);
        std::string sx, sy, sth, sc, ss_len;
        if (std::getline(ss, sx, ',') && std::getline(ss, sy, ',') && std::getline(ss, sth, ',') &&
            std::getline(ss, sc, ',') && std::getline(ss, ss_len, ',')) {
            try {
                pts.push_back({.x = std::stod(sx),
                               .y = std::stod(sy),
                               .theta = std::stod(sth),
                               .curvature = std::stod(sc),
                               .s = std::stod(ss_len)});
            } catch (...) {
                continue;
            }
        }
    }
    return pts;
}

}  // namespace benchmark
