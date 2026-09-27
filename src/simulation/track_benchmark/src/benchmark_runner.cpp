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
#include <vector>

// #19 B1：MPC 数学核（mpc_controller_lib，无 rclcpp 依赖）离线接入
#include "mpc_controller/mpc_model.hpp"
#include "track_benchmark/kpi_evaluator.hpp"
#include "track_benchmark/speed_envelope.hpp"  // #29：速度包络单一实现（PP 分支与 MPC 参考同函数）
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
    // #17：人工构造的故障算子（让“评测器本身”也被评测）。均基于仿真时间，不引入墙钟依赖，
    // 因此与正常运行一样保持位级可复现。
    //   none       正向对照
    //   offroute   持续转向偏置 → 车驶出合法走廊（应报越界）
    //   cone       更强偏置 → 车辆外廓撞锥桶（应报碰桶）
    //   reverse    沿闭合赛道反向行驶经过起点（应不计有效圈、不得产出圈速）
    //   stuck      行驶一段后永久制动驻停（未完赛：不得产出 best lap）
    std::string inject = "none";
    double freeze_after_s = 3.0;  // stuck 算子的驻停时刻（仿真时间）
    // #19 A：速度参照口径。两种剖面对**同一个控制器、同一条赛道、同一种子**跑，
    // 才能把“速度策略收益”与“控制器收益”分开（#19 验收：固定速度策略下的横向对比）。
    //   curvature 逐点按 sqrt(a_lat_max/|κ|) 限速（默认，与既有基线一致）
    //   constant  全程固定限速，不看曲率
    std::string speed_source = "curvature";
    double const_speed_mps = 5.0;  // constant 口径的固定限速
    // #19 B1：控制器选择。pure_pursuit（默认）走既有内联控制律，输出与 v1 基线
    // 逐字节一致；mpc 复用同一被控对象/赛道/KpiEvaluator，只换控制律。
    std::string controller = "pure_pursuit";
    double mpc_update_period_s = 0.02;  // MPC 解算节拍，= ROS 节点 control_rate 50Hz
};

// 开环固定舵角 (rad)：取单一侧的小角度，使车辆持续向走廊外推进。
// offroute 刚好越出 1.5m 走廊；cone 更大，确保外廓压到锥桶线。
constexpr double kOffrouteBiasRad = 0.02;
constexpr double kConeBiasRad = 0.05;

[[nodiscard]] bool known_inject(const std::string& s) {
    return s == "none" || s == "offroute" || s == "cone" || s == "reverse" || s == "stuck";
}

[[nodiscard]] bool known_speed_source(const std::string& s) {
    return s == "curvature" || s == "constant";
}

[[nodiscard]] bool known_controller(const std::string& s) {
    return s == "pure_pursuit" || s == "mpc";
}

// #19 B1：中心线 → MPC 参考路径（逐点显式速度按 --speed-source 口径，与 PP 同一剖面；
// speed_valid=true 确保 MpcModel 不回填 target_speed，两种口径真正可区分）。
// #29：限速包络走 speed_envelope.hpp 单一实现（与 PP 分支同函数），不得在此另写一份。
[[nodiscard]] std::vector<mpc::ReferencePoint> build_mpc_reference(const std::vector<CenterlinePoint>& cl,
                                                                   const std::string& speed_source, double const_speed,
                                                                   double lat_accel_max, double v_max) {
    std::vector<mpc::ReferencePoint> ref;
    ref.reserve(cl.size());
    for (const auto& p : cl) {
        const double v_curve = (speed_source == "constant")
                                   ? const_speed
                                   : benchmark::referenceSpeedForCurvature(p.curvature, lat_accel_max, v_max);
        // .curvature 传带符号值：MPC 前馈项 d(1) = -v·κ·Ts 依赖转向方向；skidpad/trackdrive 生成器
        // 已给出符号（track_generator.cpp:145），而 PP 基线口径只取幅值限速，两者不互相影响。
        ref.push_back({.x = p.x,
                       .y = p.y,
                       .theta = p.theta,
                       .curvature = p.curvature,
                       .speed = std::min(v_max, v_curve),
                       .speed_valid = true});
    }
    return ref;
}

[[nodiscard]] std::optional<TrackDefinition> make_track(const std::string& name) {
    if (name == "acceleration")
        return TrackGenerator::GenerateAcceleration();
    if (name == "skidpad")
        return TrackGenerator::GenerateSkidpad();
    if (name == "trackdrive")
        return TrackGenerator::GenerateTrackdriveLoop();
    return std::nullopt;
}

// 从 idx 起沿参考线搜索第一个距车 >= Ld 的点索引（闭合则回绕）。dir=-1 为反向行驶场景。
[[nodiscard]] size_t lookahead_index(const std::vector<CenterlinePoint>& cl, const VehicleState& s, size_t idx,
                                     double ld, bool closed, int dir = 1) {
    const size_t n = cl.size();
    for (size_t k = 1; k < n; ++k) {
        const size_t j = closed ? ((idx + static_cast<size_t>(dir * static_cast<int>(k)) + n * n) % n)
                                : (dir > 0 ? std::min(idx + k, n - 1) : idx - std::min(k, idx));
        const double dx = cl[j].x - s.x;
        const double dy = cl[j].y - s.y;
        if (std::hypot(dx, dy) >= ld)
            return j;
        if (!closed && j == n - 1)
            return j;
    }
    return closed ? (idx + static_cast<size_t>(dir)) % n : n - 1;
}

struct RunResult {
    KpiSummary summary;
    std::string run_status;  // finished | timeout | diverged
    double elapsed = 0.0;
    bool ok = false;
};

RunResult run(const RunnerConfig& cfg) {
    RunResult res;
    if (!known_inject(cfg.inject)) {
        res.run_status = "bad_inject";
        return res;
    }
    if (!known_speed_source(cfg.speed_source)) {
        res.run_status = "bad_speed_source";
        return res;
    }
    if (!known_controller(cfg.controller)) {
        res.run_status = "bad_controller";
        return res;
    }
    const bool reverse = (cfg.inject == "reverse");
    auto track = make_track(cfg.track);
    if (!track) {
        res.run_status = "bad_track";
        return res;
    }
    if (reverse && !track->closed_circuit) {
        // 反向行驶只对闭合赛道有意义（直线反跑不算“未经过起点”，判据不干净）。
        res.run_status = "bad_inject";
        return res;
    }
    const bool closed = track->closed_circuit;  // 由赛道定义指定，不靠名字猜（#17 单一来源）
    const auto& cl = track->centerline;
    const double total = track->total_length > 0.0 ? track->total_length : cl.back().s;
    const double corridor_half = track->track_width * 0.5;

    VehicleParams vp{};
    BicycleModel model(vp);
    // 反向场景：初始航向翻转 180°，目标点沿 -s 方向选取 → 车辆物理地倒跑过起点线。
    const double init_theta = cl.front().theta + (reverse ? std::numbers::pi : 0.0);
    model.Reset(cl.front().x, cl.front().y, init_theta, 0.0);

    KpiEvaluator eval(cl);
    eval.SetTrackCones(track->cones);
    eval.SetCircuitGeometry(total, corridor_half, closed, 1.5, 1.0, 0.10);
    eval.SetTrackVersion(track->versionedId());

    const double steer_bias = (cfg.inject == "offroute") ? kOffrouteBiasRad
                              : (cfg.inject == "cone")   ? kConeBiasRad
                                                         : 0.0;

    double t = 0.0;
    bool reached_end = false;
    bool frozen = false;
    size_t guard = 0;
    const size_t max_steps = static_cast<size_t>(cfg.timeout_s / cfg.dt) + 10;

    // #19 B1：MPC 分支 —— 同一被控对象/赛道/KpiEvaluator，只换控制律。
    // 解算节拍 = mpc_update_period_s（50Hz，与 ROS 节点 control_rate 一致），
    // 节拍之间 ZOH 保持上一次解；求解失败保持上次指令（失败降级统计属 B2）。
    const bool use_mpc = (cfg.controller == "mpc");
    const size_t solve_every = std::max<size_t>(1, static_cast<size_t>(cfg.mpc_update_period_s / cfg.dt + 0.5));
    mpc::MpcConfig mpc_cfg{};
    mpc_cfg.system.wheelbase = vp.wheelbase;  // 被控对象真值轴距 (#19 B0 同源)
    mpc_cfg.horizon.Ts = cfg.mpc_update_period_s;
    mpc_cfg.horizon.Np = static_cast<size_t>(0.76 / cfg.mpc_update_period_s + 0.5);  // 物理时长 0.76s (D9)
    mpc_cfg.horizon.Nc = static_cast<size_t>(0.50 / cfg.mpc_update_period_s + 0.5);  // 物理时长 0.50s (D9)
    mpc_cfg.limits.max_steer_rad = vp.max_steer_angle;
    mpc_cfg.limits.max_steer_rate = vp.max_steer_rate;
    mpc_cfg.limits.min_accel = -vp.max_decel;
    mpc_cfg.limits.max_accel = vp.max_accel;
    mpc::MpcModel mpc_model(mpc_cfg);
    std::vector<mpc::ReferencePoint> mpc_ref =
        build_mpc_reference(cl, cfg.speed_source, cfg.const_speed_mps, cfg.lat_accel_max, vp.max_speed);
    if (closed) {
        // 闭环回绕：MpcModel::Step 取 best_idx 后 Np 点、无回绕语义（与 ROS 节点开路径一致，
        // 不动共享核）；离线侧把头部再拼一段，使末端时域看到的仍是真实弯道而非切线外推。
        mpc_ref.reserve(mpc_ref.size() + mpc_cfg.horizon.Np);
        for (size_t k = 0; k < mpc_cfg.horizon.Np; ++k)
            mpc_ref.push_back(mpc_ref[k]);
    }
    bool mpc_has_cmd = false;  // 首次成功求解前保持零指令
    double mpc_steer = 0.0;
    double mpc_accel = 0.0;

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

        double steer = 0.0;
        double accel = 0.0;
        if (use_mpc) {
            if (guard % solve_every == 0) {
                const auto sol = mpc_model.Step(s.x, s.y, s.theta, s.v, mpc_steer, mpc_accel, mpc_ref);
                if (sol.success) {
                    mpc_steer = sol.steering_rad;
                    mpc_accel = sol.accel_mps2;
                    mpc_has_cmd = true;
                }
                // 失败：保持上次指令（首步即失败则维持零指令）
            }
            steer = mpc_steer;
            accel = mpc_accel;
            if (mpc_has_cmd) {
                steer = std::clamp(steer, -vp.max_steer_angle, vp.max_steer_angle);
                accel = std::clamp(accel, -vp.max_decel, vp.max_accel);
            }
        } else {
            const double ld = cfg.lookahead_base + cfg.lookahead_gain * s.v;
            const size_t ti = lookahead_index(cl, s, idx, ld, closed, reverse ? -1 : 1);

            // 纯跟踪转向
            const double dx = cl[ti].x - s.x;
            const double dy = cl[ti].y - s.y;
            const double lx = dx * std::cos(s.theta) + dy * std::sin(s.theta);
            const double ly = -dx * std::sin(s.theta) + dy * std::cos(s.theta);
            const double d = std::max(std::hypot(dx, dy), 1e-3);
            const double alpha = std::atan2(ly, lx);
            steer = std::atan(2.0 * vp.wheelbase * std::sin(alpha) / d);
            steer = std::clamp(steer, -vp.max_steer_angle, vp.max_steer_angle);

            // 曲率限速 + P 控制器（--speed-source constant 时改用固定限速，作公平对比的另一口径）
            // #29：限速包络走 speed_envelope.hpp 单一实现（与 MPC 参考构建同函数）。
            const double v_curve =
                (cfg.speed_source == "constant")
                    ? cfg.const_speed_mps
                    : benchmark::referenceSpeedForCurvature(cl[idx].curvature, cfg.lat_accel_max, vp.max_speed);
            const double v_ref = std::min(vp.max_speed, v_curve);
            accel = std::clamp(cfg.speed_kp * (v_ref - s.v), -vp.max_decel, vp.max_accel);
        }

        // 故障注入（均基于仿真时间，位级可复现）
        if (cfg.inject == "stuck" && t >= cfg.freeze_after_s)
            frozen = true;
        if (frozen) {
            steer = 0.0;
            accel = -vp.max_decel;  // 持续制动直至停稳
        } else if (steer_bias != 0.0) {
            // 开环固定舵角，而不是在闭环输出上叠偏置：后者会被循迹反馈吸收，
            // 无法稳定构造“持续驶出走廊 / 撞桶”的负样本。
            steer = std::clamp(steer_bias, -vp.max_steer_angle, vp.max_steer_angle);
        }

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
    summary.controller_name = use_mpc ? "MPC" : "PurePursuit";  // #19 B1：MPC 路径标记；PP 与基线常量同值
    summary.elapsed_s = t;
    const bool laps_ok = !closed || cfg.require_laps == 0 || summary.valid_laps >= cfg.require_laps;
    const bool rmse_ok = !(summary.total_samples > 0 && summary.rmse_lateral_m > cfg.rmse_max);
    // 反向/越界/碰桶/驻停等故障场景必须回落到未达标（laps_ok 由有效圈保证，rmse_ok 由走廊误差保证）。
    if (closed) {
        summary.finished = laps_ok && rmse_ok;
        summary.run_status = summary.finished ? "finished" : (t >= cfg.timeout_s ? "timeout" : "incomplete");
    } else {
        summary.finished = reached_end && rmse_ok;
        summary.run_status = summary.finished ? "finished" : (t >= cfg.timeout_s ? "timeout" : "incomplete");
    }
    // 让 timed_out 与 run_status 保持一致（此前恒为 false，与 run_status="timeout" 自相矛盾）。
    summary.timed_out = (summary.run_status == "timeout");
    res.summary = summary;
    res.elapsed = t;
    res.run_status = summary.run_status;
    res.ok = summary.finished;
    return res;
}

void print_usage() {
    std::cout << "Usage: benchmark_runner [--track acceleration|skidpad|trackdrive] [--dt S]\n"
              << "       [--require-laps N] [--timeout S] [--rmse-max M] [--lookahead-base M]\n"
              << "       [--inject none|offroute|cone|reverse|stuck] [--freeze-after S]\n"
              << "       [--speed-source curvature|constant] [--const-speed MPS]\n"
              << "       [--controller pure_pursuit|mpc] [--mpc-update-period S]\n"
              << "       [--out FILE.json]\n"
              << "       [--export-tracks DIR]   # 导出唯一赛道来源（CSV + tracks.json）后退出\n"
              << "  --inject: #17 人工构造故障场景（退出码非 0，并应在 JSON 里命中对应判据字段）\n"
              << "  --speed-source: #19 A 速度口径（constant = 全程固定限速，与 curvature 同控制器同赛道对比）\n"
              << "  --controller: #19 B1 控制律（mpc = 离线 MPC 数学核；默认 pure_pursuit 与 v1 基线位级一致）\n";
}

// #17：把生成器几何导出为仿真器可直接消费的 CSV + 清单。回归脚本拿它与仓内已提交 CSV 做
// 逐字节 diff：任何一侧单独改动都会被查出，“名字相同、几何不同”不再可能悄悄发生。
int exportTracks(const std::string& dir) {
    const std::vector<TrackDefinition> tracks = {TrackGenerator::GenerateAcceleration(),
                                                 TrackGenerator::GenerateSkidpad(),
                                                 TrackGenerator::GenerateTrackdriveLoop()};
    const std::vector<std::string> files = {"acceleration_track.csv", "skidpad_track.csv", "trackdrive_loop.csv"};
    for (size_t i = 0; i < tracks.size(); ++i) {
        if (!TrackGenerator::SaveConesToCSV(tracks[i], dir + "/" + files[i])) {
            std::cerr << "ERROR: cannot write " << dir + "/" + files[i] << "\n";
            return 2;
        }
        std::cerr << "[export] " << files[i] << " cones=" << tracks[i].cones.size() << " geometry_fnv1a64=" << std::hex
                  << TrackGenerator::geometryChecksum(tracks[i].cones) << std::dec << "\n";
    }
    if (!TrackGenerator::SaveTrackManifest(tracks, dir, files)) {
        std::cerr << "ERROR: cannot write " << dir + "/tracks.json\n";
        return 2;
    }
    return 0;
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
        else if (a == "--inject")
            cfg.inject = next("none");
        else if (a == "--freeze-after")
            cfg.freeze_after_s = std::stod(next("3"));
        else if (a == "--speed-source")
            cfg.speed_source = next("curvature");
        else if (a == "--controller")
            cfg.controller = next("pure_pursuit");
        else if (a == "--mpc-update-period")
            cfg.mpc_update_period_s = std::stod(next("0.02"));
        else if (a == "--const-speed")
            cfg.const_speed_mps = std::stod(next("5"));
        else if (a == "--out")
            cfg.out = next("");
        else if (a == "--export-tracks") {
            // 导完即退：不跑任何场景，给回归脚本/标定流程一个确定的导出入口。
            return exportTracks(next(""));
        } else if (a == "-h" || a == "--help") {
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
    std::cerr << "[benchmark_runner] track=" << cfg.track << " controller=" << cfg.controller
              << " inject=" << cfg.inject << " speed_source=" << cfg.speed_source << " status=" << res.run_status
              << " laps=" << res.summary.valid_laps << " rmse=" << res.summary.rmse_lateral_m << "\n";
    return res.ok ? 0 : 1;
}
