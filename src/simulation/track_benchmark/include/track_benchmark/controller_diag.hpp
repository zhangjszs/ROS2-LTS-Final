#pragma once
// ============================================================================
// #44（#19 B2）：控制器诊断采集层 —— 把"solve_time 分布 / 未收敛率 / 降级次数"
// 变成可机读产物（纯 std，无 ROS，可脱 colcon 编译并进 ASan+UBSan 门）。
//
// 为什么单独一层：#19 协议要求"失败降级纳入"与"结论只覆盖实际验证条件"，
// 而 fsac.benchmark.kpi/v1 的 JSON 字段集是三条 v1 基线的口径锚点（见
// docs/INTERFACE_CONTRACT.md §4），**不能**往里加字段。因此诊断走独立产物
// schema fsac.benchmark.controller_diag/v1，由 runner 的 --diag-out 落盘。
//
// 计数口径（两臂必须同构，否则 PP/MPC 不可对照）：
//   control_ticks    控制律被请求给出指令的时点数。
//                    PP = 每个仿真步；MPC = 每个解算节拍（不是每一步，
//                    节拍之间的 ZOH 保持是一次解的正常使用，**不算降级**）。
//   command_updates  控制律真的给出了新指令并被采用的时点数。
//   holds            控制律没能给出新指令、沿用上一条的时点数（MPC 求解失败；
//                    注入类开环接管）。不变式：command_updates + holds == control_ticks。
//   solves/failures  仅 MPC 有（PP 是解析律，无求解过程）；failure_rate = failures/solves。
//
// 百分位定义（写死，避免实现者各自解释）：升序排序后取**最近秩**
//   idx = ceil(q/100 · n)，1-based，结果一定是某个真实样本值（不做线性插值，
//   不产生样本之外的"漂亮数字"）。n == 0 时返回 0.0，并由 sample_count=0 表达
//   "无数据"——调用方不得把 0 读成"耗时为零"。
// ============================================================================
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <format>
#include <string>
#include <vector>

namespace benchmark {

class ControllerDiagnostics {
   public:
    struct Snapshot {
        // 字段尾注一律放独立行：规避 clang-format v18(CI)/v21(本地) 的对齐分组差异。
        std::uint64_t control_ticks{0};
        std::uint64_t command_updates{0};
        std::uint64_t holds{0};
        std::uint64_t solves{0};
        std::uint64_t failures{0};
        std::uint64_t sample_count{0};
        double failure_rate{0.0};
        double mean_ms{0.0};
        double p50_ms{0.0};
        double p95_ms{0.0};
        double p99_ms{0.0};
        double max_ms{0.0};
    };

    void RecordControlTick() noexcept { ++control_ticks_; }
    void RecordCommandUpdate() noexcept { ++command_updates_; }
    void RecordHold() noexcept { ++holds_; }

    /**
     * @brief 记录一次求解（仅 MPC 类迭代求解器调用）
     * @param elapsed_ms  核内算出的求解耗时；非有限或为负则**不进入分布样本**，
     *                    但 solves/failures 仍计数（耗时不可信 ≠ 求解没发生）
     * @param success       求解是否收敛（false 即一次未收敛）
     */
    void RecordSolve(double elapsed_ms, bool success) noexcept {
        ++solves_;
        if (!success) {
            ++failures_;
        }
        if (std::isfinite(elapsed_ms) && elapsed_ms >= 0.0) {
            samples_.push_back(elapsed_ms);
        }
    }

    [[nodiscard]] Snapshot GetSnapshot() const {
        Snapshot s;
        s.control_ticks = control_ticks_;
        s.command_updates = command_updates_;
        s.holds = holds_;
        s.solves = solves_;
        s.failures = failures_;
        s.sample_count = samples_.size();
        s.failure_rate = (solves_ > 0) ? static_cast<double>(failures_) / static_cast<double>(solves_) : 0.0;
        if (!samples_.empty()) {
            std::vector<double> sorted = samples_;
            std::sort(sorted.begin(), sorted.end());
            double sum = 0.0;
            for (const double v : sorted) {
                sum += v;
            }
            s.mean_ms = sum / static_cast<double>(sorted.size());
            s.p50_ms = PercentileOfSorted(sorted, 50.0);
            s.p95_ms = PercentileOfSorted(sorted, 95.0);
            s.p99_ms = PercentileOfSorted(sorted, 99.0);
            s.max_ms = sorted.back();
        }
        return s;
    }

    /**
     * @brief 最近秩百分位（要求 samples 已升序）：idx = ceil(q/100·n) 取 1-based 第 idx 个
     * @return 空集合或 q 非有限时返回 0.0（配合 sample_count 判"无数据"）
     */
    [[nodiscard]] static double PercentileOfSorted(const std::vector<double>& samples, double q) noexcept {
        const std::size_t n = samples.size();
        if (n == 0 || !std::isfinite(q)) {
            return 0.0;
        }
        const double clamped = std::clamp(q, 0.0, 100.0);
        std::size_t idx = static_cast<std::size_t>(std::ceil(clamped / 100.0 * static_cast<double>(n)));
        idx = std::clamp(idx, std::size_t{1}, n);
        return samples[idx - 1];
    }

    /// 机读诊断产物（字段顺序固定 → 同一输入两次运行位级一致）
    [[nodiscard]] std::string GenerateJson(std::string_view track_version, std::string_view controller_name,
                                           std::string_view speed_source) const {
        const Snapshot s = GetSnapshot();
        std::string j;
        j += "{\n";
        j += "  \"schema\": \"fsac.benchmark.controller_diag/v1\",\n";
        j += "  \"percentile_method\": \"nearest_rank_ceil\",\n";
        j += "  \"track_version\": \"" + JsonEscape(track_version) + "\",\n";
        j += "  \"controller_name\": \"" + JsonEscape(controller_name) + "\",\n";
        j += "  \"speed_source\": \"" + JsonEscape(speed_source) + "\",\n";
        j += "  \"control_ticks\": " + std::to_string(s.control_ticks) + ",\n";
        j += "  \"command_updates\": " + std::to_string(s.command_updates) + ",\n";
        j += "  \"holds\": " + std::to_string(s.holds) + ",\n";
        j += "  \"solves\": " + std::to_string(s.solves) + ",\n";
        j += "  \"failures\": " + std::to_string(s.failures) + ",\n";
        j += std::format("  \"failure_rate\": {:.6},\n", s.failure_rate);
        j += "  \"sample_count\": " + std::to_string(s.sample_count) + ",\n";
        j += std::format("  \"mean_solve_ms\": {:.6},\n", s.mean_ms);
        j += std::format("  \"p50_solve_ms\": {:.6},\n", s.p50_ms);
        j += std::format("  \"p95_solve_ms\": {:.6},\n", s.p95_ms);
        j += std::format("  \"p99_solve_ms\": {:.6},\n", s.p99_ms);
        j += std::format("  \"max_solve_ms\": {:.6}\n", s.max_ms);
        j += "}\n";
        return j;
    }

   private:
    [[nodiscard]] static std::string JsonEscape(std::string_view v) {
        std::string out;
        out.reserve(v.size() + 8);
        for (const char c : v) {
            if (c == '"' || c == '\\') {
                out += '\\';
                out += c;
            } else if (c == '\n') {
                out += "\\n";
            } else {
                out += c;
            }
        }
        return out;
    }

    std::uint64_t control_ticks_{0};
    std::uint64_t command_updates_{0};
    std::uint64_t holds_{0};
    std::uint64_t solves_{0};
    std::uint64_t failures_{0};
    std::vector<double> samples_;
};

}  // namespace benchmark
