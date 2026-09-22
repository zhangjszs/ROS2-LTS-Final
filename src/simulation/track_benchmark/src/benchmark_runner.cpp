// #17-B：离线确定性核心回归 runner（不依赖 ROS 调度/DDS，纯定步长积分）。
// 组合 真实车辆模型 (simulation::BicycleModel) + 纯跟踪控制器 + KpiEvaluator，
// 跑一个代表性场景并产出机读 JSON。用于 CI 的“算法核心确定性检查”：固定配置/赛道/
// 步长时输出位级可复现，快速、稳定、无时序抖动。ROS 闭环集成冒烟（含调度、需容差）另设。
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>

#include "track_benchmark/kpi_evaluator.hpp"
#include "track_benchmark/track_generator.hpp"
#include "vehicle_simulator/bicycle_model.hpp"

namespace {

using benchmark::CenterlinePoint;
using benchmark::KpiEvaluator;
using benchmark::KpiSummary;
using benchmark::TrackDefinition;
using benchmark::TrackGenerator;
using simulation::BicycleModel;
using simulation::ControlCommand;
using simulation::VehicleParams;
using simulation::VehicleState;

struct RunnerConfig {
    std::string track = "acceleration";  // acceleration | skidpad | trackdrive
    double dt = 0.01;                    // 定步长 (s)
    double lookahead_base = 2.5;         // 纯跟踪基础前视 (m)
    double lookahead_gain = 0.35;        // 前视随速度增益 (s)
    double lat_accel_max = 4.0;          // 速度规划侧向加速度上限 (m/s^2)
    double speed_kp = 2.0;               // 速度跟踪比例
    int require_laps = 0;                // 闭合赛道需完成的有效圈数 (>0 时生效)
    double timeout_s = 120.0;            // 仿真时间上界 (s)
    double rmse_max = 1.5;               // 横向 RMSE 通过门限 (m)
    std::string out = "";                // JSON 输出路径
};

[[nodiscard]] std::optional<TrackDefinition> make_track(const std::string& name) {
    if (name == "acceleration")
        return TrackGenerator::GenerateAcceleration();
    if (name == "skidpad")
        return TrackGenerator::GenerateSkidpad();
    if (name == "trackdrive")
        return TrackGenerator::GenerateTrackdriveLoop();
    return std::nullopt;
}

// 从 idx 起沿参考线向前搜索第一个距车 >= Ld 的点索引（闭合则回绕）。
[[nodiscard]] size_t lookahead_index(const std::vector<CenterlinePoint>& cl, const VehicleState& s, size_t idx,
                                     double ld, bool closed) {
    const size_t n = cl.size();
    for (size_t k = 1; k < n; ++k) {
        size_t j = closed ? (idx + k) % n : std::min(idx + k, n - 1);
        const double dx = cl[j].x - s.x;
        const double dy = cl[j].y - s.y;
        if (std::hypot(dx, dy) >= ld)
            return j;
        if (!closed && j == n - 1)
            return j;
    }
    return closed ? (idx + 1) % n : n - 1;
}

struct RunResult {
    KpiSummary summary;
    std::string run_status;  // finished | timeout | diverged
    double elapsed = 0.0;
    bool ok = false;
};

RunResult run(const RunnerConfig& cfg) {
    RunResult res;
    auto track = make_track(cfg.track);
    if (!track) {
        res.run_status = "bad_track";
        return res;
    }
    const bool closed = (cfg.track != "acceleration");
    const auto& cl = track->centerline;
    const double total = track->total_length > 0.0 ? track->total_length : cl.back().s;
    const double corridor_half = track->track_width * 0.5;

    VehicleParams vp{};
    BicycleModel model(vp);
    model.Reset(cl.front().x, cl.front().y, cl.front().theta, 0.0);

    KpiEvaluator eval(cl);
    eval.SetTrackCones(track->cones);
    eval.SetCircuitGeometry(total, corridor_half, closed, 1.5, 1.0, 0.10);
    eval.SetTrackVersion(track->name + "/v1");

    double t = 0.0;
    bool reached_end = false;
    size_t guard = 0;
    const size_t max_steps = static_cast<size_t>(cfg.timeout_s / cfg.dt) + 10;

    while (t <= cfg.timeout_s && guard++ < max_steps) {
        const VehicleState s = model.state();
        if (!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.theta)) {
            res.run_status = "diverged";
            res.elapsed = t;
            res.summary = eval.GetSummary();
            return res;
        }

        size_t idx = 0;
        eval.ComputeCrossTrackError(s.x, s.y, s.theta, &idx);
        const double ld = cfg.lookahead_base + cfg.lookahead_gain * s.v;
        const size_t ti = lookahead_index(cl, s, idx, ld, closed);

        // 纯跟踪转向
        const double dx = cl[ti].x - s.x;
        const double dy = cl[ti].y - s.y;
        const double lx = dx * std::cos(s.theta) + dy * std::sin(s.theta);
        const double ly = -dx * std::sin(s.theta) + dy * std::cos(s.theta);
        const double d = std::max(std::hypot(dx, dy), 1e-3);
        const double alpha = std::atan2(ly, lx);
        double steer = std::atan(2.0 * vp.wheelbase * std::sin(alpha) / d);
        steer = std::clamp(steer, -vp.max_steer_angle, vp.max_steer_angle);

        // 曲率限速 + P 控制器
        const double kappa = std::abs(cl[idx].curvature);
        const double v_curve = std::sqrt(cfg.lat_accel_max / std::max(kappa, 1e-3));
        const double v_ref = std::min(vp.max_speed, v_curve);
        double accel = std::clamp(cfg.speed_kp * (v_ref - s.v), -vp.max_decel, vp.max_accel);

        ControlCommand cmd{.target_steering = steer, .target_accel = accel};
        model.Step(cmd, cfg.dt);
        t += cfg.dt;
        const VehicleState& ns = model.state();
        eval.Update(ns.x, ns.y, ns.theta, ns.v, ns.steering_angle, t);

        if (!closed && idx + 1 >= cl.size()) {
            reached_end = true;
            break;
        }
        if (closed && cfg.require_laps > 0 && eval.GetSummary().valid_laps >= cfg.require_laps) {
            break;
        }
    }

    KpiSummary summary = eval.GetSummary();
    summary.track_name = track->name;
    summary.elapsed_s = t;
    const bool laps_ok = !closed || cfg.require_laps == 0 || summary.valid_laps >= cfg.require_laps;
    const bool rmse_ok = !(summary.total_samples > 0 && summary.rmse_lateral_m > cfg.rmse_max);
    if (closed) {
        summary.finished = laps_ok && rmse_ok;
        summary.run_status = summary.finished ? "finished" : (t >= cfg.timeout_s ? "timeout" : "running");
    } else {
        summary.finished = reached_end && rmse_ok;
        summary.run_status = summary.finished ? "finished" : "timeout";
    }
    res.summary = summary;
    res.elapsed = t;
    res.run_status = summary.run_status;
    res.ok = summary.finished;
    return res;
}

void print_usage() {
    std::cout << "Usage: benchmark_runner [--track acceleration|skidpad|trackdrive] [--dt S]\n"
              << "       [--require-laps N] [--timeout S] [--rmse-max M] [--lookahead-base M]\n"
              << "       [--out FILE.json]\n";
}

}  // namespace

int main(int argc, char** argv) {
    RunnerConfig cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char* dflt) { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(dflt); };
        if (a == "--track")
            cfg.track = next("acceleration");
        else if (a == "--dt")
            cfg.dt = std::stod(next("0.01"));
        else if (a == "--require-laps")
            cfg.require_laps = std::stoi(next("0"));
        else if (a == "--timeout")
            cfg.timeout_s = std::stod(next("120"));
        else if (a == "--rmse-max")
            cfg.rmse_max = std::stod(next("1.5"));
        else if (a == "--lookahead-base")
            cfg.lookahead_base = std::stod(next("2.5"));
        else if (a == "--out")
            cfg.out = next("");
        else if (a == "-h" || a == "--help") {
            print_usage();
            return 0;
        }
    }

    const RunResult res = run(cfg);
    const std::string json = KpiEvaluator::GenerateJsonReport(res.summary);
    std::cout << json;
    if (!cfg.out.empty()) {
        std::ofstream out(cfg.out);
        if (out.is_open())
            out << json;
        else
            std::cerr << "WARN: cannot write " << cfg.out << "\n";
    }
    std::cerr << "[benchmark_runner] track=" << cfg.track << " status=" << res.run_status
              << " laps=" << res.summary.valid_laps << " rmse=" << res.summary.rmse_lateral_m << "\n";
    return res.ok ? 0 : 1;
}
