#pragma once
// ============================================================================
// issue #16：最终指令仲裁层 —— 多个控制指令源（Pure Pursuit / MPC / safety 降级）
// 仲裁出**唯一**最终底盘指令（单一事实来源），并保证安全降级输出确定性。
// 纯 std 依赖（无 ROS context，可单测）；复用既有契约层，不重复实现判定原语：
//   - common_msgs::contract::sourceAgeAcceptable（#12 输入陈旧/超时检测）
//   - common_msgs::vehicle::verifyChecksum / checksumRaw / verifyFrame（#15 指令帧有效性）
//   - common_msgs::vehicle::ActuatorCalibration（#15 clamp-before-narrow / 非有限降级）
//   - common_msgs::vehicle::SafetyState / TaskSafetyStateMachine（#16 状态）
//
// 优先级与抢占语义（高→低）：
//   1) 安全/故障态（stopActive 或状态机 STOP）：强制“无油门 + 安全制动 + 零转角”，最高，不可被控制源覆盖；
//   2) 任务态约束：非 canDrive（IDLE/未武装 或 已停）→ 输出安全制动而非行驶指令；
//   3) 正常控制源选择：仅在 present + 帧头合法 + 线上 checksum 有效 + 新鲜的候选中，按 preferred 选；冲突按 preferred，
//      无任一可信候选 → 安全降级（NO_TRUSTED_SOURCE）。
//
// 说明：滞回/防抖（切换驻留时间 dwell）需要跨帧记忆，decide() 为**纯函数**（不持时间），
// 去抖/切换计数由上层接线节点维护；本头提供确定性、可注入故障的单帧判定，便于回归门禁。
// ============================================================================
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include "interface_contract.h"     // #14：sourceAgeAcceptable 等判定原语
#include "task_state_machine.h"     // #16：SafetyState / TaskSafetyStateMachine
#include "vehicle_command_codec.h"  // #15：VehicleCommandRaw / checksum / ActuatorCalibration

namespace common_msgs {
namespace vehicle {

enum class ControlSource : std::uint8_t { NONE = 0, PURE_PURSUIT = 1, MPC = 2 };

// 单个控制源的候选指令 + 其可信度元数据（年龄由上层按单调时钟算好传入，保证纯函数可测）。
struct SourceCandidate {
    ControlSource source = ControlSource::NONE;
    VehicleCommandRaw cmd{};
    std::uint16_t checksum = 0;
    double age_sec = 0.0;
    bool present = false;
};

struct ArbitrationConfig {
    // 单源输入新鲜度容差（秒）；<0 表示禁用新鲜度检查（仅校验存在 + checksum）。
    double source_timeout_sec = 0.5;
    // 期望控制源（冲突时优先）；不可用时回退到另一可信源（按枚举顺序，确定性）。
    ControlSource preferred = ControlSource::PURE_PURSUIT;
    // 切换驻留（去抖）时间（秒）：新源须连续可信 >=dwell 才能抢走当前 winner；0 表示无去抖。
    double switch_dwell_sec = 0.0;
};

enum class ArbitrationReason {
    CONTROL,            // 正常控制源胜出
    STOP_ACTIVE,        // 安全/故障态 → 强制停车
    TASK_NOT_DRIVING,   // 任务态不允许行驶 → 安全制动
    NO_TRUSTED_SOURCE,  // 无 present+新鲜+checksum 有效的候选 → 安全降级
};

struct ArbitrationResult {
    ControlSource winner = ControlSource::NONE;
    VehicleCommandRaw cmd{};
    std::uint16_t checksum = 0;
    bool safe_fallback = false;
    ArbitrationReason reason = ArbitrationReason::NO_TRUSTED_SOURCE;
};

// 单帧决策。cal/neutral_steering 用于构造确定性安全输出（复用 #15 语义，绝不越界/负值→255）。
class CommandArbitrator {
   public:
    CommandArbitrator(ActuatorCalibration cal, std::uint8_t neutral_steering)
        : cal_(cal), neutral_steering_(neutral_steering) {}

    // 依据统一状态机当前状态决策：STOP / 非 canDrive 均落到安全制动。
    ArbitrationResult decide(const TaskSafetyStateMachine& sm, std::span<const SourceCandidate> candidates,
                             const ArbitrationConfig& cfg) const {
        if (sm.stopActive()) {
            return safeStopResult(ArbitrationReason::STOP_ACTIVE);
        }
        if (!sm.canDrive()) {
            return safeStopResult(ArbitrationReason::TASK_NOT_DRIVING);
        }
        return decideRunning(candidates, cfg);
    }

    // 显式传入安全标志（便于在状态机之外复用，如 safety_monitor 直接喂 StopStateMachine 结果）。
    ArbitrationResult decide(bool stop_active, bool can_drive, std::span<const SourceCandidate> candidates,
                             const ArbitrationConfig& cfg) const {
        if (stop_active) {
            return safeStopResult(ArbitrationReason::STOP_ACTIVE);
        }
        if (!can_drive) {
            return safeStopResult(ArbitrationReason::TASK_NOT_DRIVING);
        }
        return decideRunning(candidates, cfg);
    }

    // 确定性安全停车指令：零转角 + 无油门 + 锁定制动；checksum 同源计算。
    ArbitrationResult safeStopResult(ArbitrationReason reason) const {
        VehicleCommandRaw raw;
        raw.steering = clampSteering(neutral_steering_);
        raw.pedal_ratio = 0;
        raw.brake_force = cal_.emergencyBrakeRaw();
        raw.gear_position = 1;
        raw.working_mode = 1;
        ArbitrationResult r;
        r.winner = ControlSource::NONE;
        r.cmd = raw;
        r.checksum = checksumRaw(raw);
        r.safe_fallback = true;
        r.reason = reason;
        return r;
    }

   private:
    ArbitrationResult decideRunning(std::span<const SourceCandidate> candidates, const ArbitrationConfig& cfg) const {
        // 收集可信候选：present + checksum 有效 + 新鲜（sourceAgeAcceptable；容差<0 视为禁用=接受）。
        const SourceCandidate* best = nullptr;
        for (const auto& c : candidates) {
            if (!c.present)
                continue;
            if (!verifyChecksum(c.cmd, c.checksum))
                continue;  // checksum 失败 → 视为不可信，忽略该源
            if (!trustedAge(c.age_sec, cfg.source_timeout_sec))
                continue;  // 陈旧/超时 → 忽略
            if (c.source == ControlSource::NONE)
                continue;
            if (c.source == cfg.preferred) {
                best = &c;  // 期望源可信：直接胜出
                break;
            }
            if (best == nullptr)
                best = &c;  // 否则退而取首个可信源（枚举顺序由上层传入顺序决定，保持确定性）
        }

        if (best == nullptr) {
            return safeStopResult(ArbitrationReason::NO_TRUSTED_SOURCE);
        }

        ArbitrationResult r;
        r.winner = best->source;
        r.cmd = best->cmd;
        r.checksum = checksumRaw(best->cmd);  // 以共用层重算，防调用方 checksum 漂移
        r.safe_fallback = false;
        r.reason = ArbitrationReason::CONTROL;
        return r;
    }

    static bool trustedAge(double age_sec, double tolerance_sec) {
        // 容差<0 => 禁用新鲜度检查（接受）；否则要求非负且不超龄（#12 语义）。
        if (tolerance_sec < 0.0)
            return true;
        return contract::sourceAgeAcceptable(age_sec, tolerance_sec);
    }

    static std::uint8_t clampSteering(std::uint8_t v) noexcept { return v; }

    ActuatorCalibration cal_;
    std::uint8_t neutral_steering_;
};

// ── #16：跨帧有状态封装（去抖/切换驻留 + stale 老化），保持确定性可测 ──────────────
// 与无状态 CommandArbitrator 不同：本类维护当前 winner 与各源“连续可信起始时刻”，
// 需一个**单调时钟** now_sec 由调用方（节点用 ROS clock / 单测用虚拟时钟）逐帧传入。
// 语义：
//   · 安全/不可行驶 → 输出安全降级并清空 winner；
//   · 当前 winner 仍去抖合格 → 保持（sticky，不因瞬时抖动切走）；
//   · 否则在“连续可信 >= switch_dwell_sec”的源中选 preferred，无合格源则降级。
class CommandArbiterFilter {
   public:
    explicit CommandArbiterFilter(CommandArbitrator arb) : arb_(arb) { reset(); }

    // 一帧观测：last_rx_sec = 该源最近一次收到指令的单调时刻（<0 = 从未收到）。
    // checksum 必须是**线上帧自带值**（在接线层原样透传）：若在本层重算，verifyChecksum 将永真，
    // “篡改/截断帧不可信”这道门事实上失效（#16 已发生过的接线缺陷，由单测锁定）。
    struct SourceObs {
        ControlSource source = ControlSource::NONE;
        VehicleCommandRaw cmd{};
        std::uint16_t checksum = 0;
        double last_rx_sec = -1.0;
        bool present = false;
        // 帧头/长度合法性（verifyFrame，#15）。默认 false = 安全侧失效：忘了显式判定的接线
        // 只会得到安全降级，不会静默接受一个来历不明的指令帧。
        bool frame_valid = false;
    };

    ControlSource winner() const noexcept { return winner_; }

    // 单源不可信的具体原因（#16 遥测/计数用）。与 update() 内的 trusted 判定**同一实现**，
    // 保证接线层的拒收统计与实际门不会漂移。
    // 枚举项尾注一律改为独立注释行：规避 clang-format v18(CI)/v21(本地) 对齐差异。
    enum class Untrusted : std::uint8_t {
        // 可信
        kNone = 0,
        // 从未收到
        kAbsent,
        // 帧头/长度不合法（#15 verifyFrame）
        kBadFrame,
        // 线上校验和与指令区不一致（篡改/截断）
        kBadChecksum,
        // 超过 source_timeout_sec（#12 新鲜度）
        kStale,
    };

    static Untrusted classify(const SourceObs& o, double now_sec, const ArbitrationConfig& cfg) {
        if (!o.present || o.source == ControlSource::NONE || o.last_rx_sec < 0.0)
            return Untrusted::kAbsent;
        if (!o.frame_valid)
            return Untrusted::kBadFrame;
        if (!verifyChecksum(o.cmd, o.checksum))
            return Untrusted::kBadChecksum;
        if (!trustedAge(now_sec - o.last_rx_sec, cfg.source_timeout_sec))
            return Untrusted::kStale;
        return Untrusted::kNone;
    }

    // 确定性安全降级指令（供节点启动/预热时直接发布，使最终出口一开始就有合法生产者）。
    ArbitrationResult safeStop(ArbitrationReason reason) const { return arb_.safeStopResult(reason); }

    ArbitrationResult update(double now_sec, const TaskSafetyStateMachine& sm, std::span<const SourceObs> obs,
                             const ArbitrationConfig& cfg) {
        return update(now_sec, sm.stopActive(), sm.canDrive(), obs, cfg);
    }

    ArbitrationResult update(double now_sec, bool stop_active, bool can_drive, std::span<const SourceObs> obs,
                             const ArbitrationConfig& cfg) {
        if (stop_active || !can_drive) {
            reset();
            return arb_.safeStopResult(stop_active ? ArbitrationReason::STOP_ACTIVE
                                                   : ArbitrationReason::TASK_NOT_DRIVING);
        }
        // 逐源更新“连续可信起始时刻”：瞬时失效则重置为 NaN（需重新累计 dwell）。
        for (const auto& o : obs) {
            const std::size_t idx = index(o.source);
            if (idx >= kMaxSources)
                continue;
            const bool trusted = classify(o, now_sec, cfg) == Untrusted::kNone;
            if (trusted) {
                if (std::isnan(valid_since_[idx]))
                    valid_since_[idx] = now_sec;
                last_valid_[idx] = o;  // 缓存最新有效指令
            } else {
                valid_since_[idx] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        // 可信判定：valid_since 非 NaN 表示“本帧可信”（不信任时已重置为 NaN）。
        auto trustedNow = [&](ControlSource s) {
            const std::size_t idx = index(s);
            return idx < kMaxSources && !std::isnan(valid_since_[idx]);
        };
        // 去抖合格：本帧可信 且 连续可信时长 >= dwell（仅用于“切换/接管”防抖）。
        auto debouncedOk = [&](ControlSource s) {
            const std::size_t idx = index(s);
            return trustedNow(s) && (now_sec - valid_since_[idx]) + 1e-9 >= cfg.switch_dwell_sec;
        };
        // 在 obs 中挑首选（preferred 优先，其次枚举顺序）；谓词决定即时接管或需去抖。
        auto pick = [&](auto pred) {
            if (pred(cfg.preferred))
                return cfg.preferred;
            for (const auto& o : obs) {
                if (pred(o.source))
                    return o.source;
            }
            return ControlSource::NONE;
        };

        // 优先级：已 committed 且当前可信 → 保持（sticky，不需 dwell）；
        // 冷启动（winner_==NONE）→ 立即接管任一可信源（无需 dwell）；
        // 从已失效的 committed 源切换 → 需候选去抖合格（防抓不住一个抖动的新源）。
        ControlSource chosen = ControlSource::NONE;
        const bool sticky = (winner_ != ControlSource::NONE) && trustedNow(winner_);
        if (sticky) {
            chosen = winner_;
        } else if (winner_ == ControlSource::NONE) {
            chosen = pick(trustedNow);
        } else {
            chosen = pick(debouncedOk);
        }

        if (chosen == ControlSource::NONE) {
            // 未发生切换时保留旧 winner（让 dwell 在失效 committed 源后继续累计），冷启动则维持 NONE。
            return arb_.safeStopResult(ArbitrationReason::NO_TRUSTED_SOURCE);
        }
        winner_ = chosen;
        const SourceCandidate cand = toCandidate(last_valid_[index(chosen)]);
        ArbitrationResult r;
        r.winner = chosen;
        r.cmd = cand.cmd;
        r.checksum = checksumRaw(cand.cmd);
        r.safe_fallback = false;
        r.reason = ArbitrationReason::CONTROL;
        return r;
    }

    void reset() noexcept {
        winner_ = ControlSource::NONE;
        for (std::size_t i = 0; i < kMaxSources; ++i)
            valid_since_[i] = std::numeric_limits<double>::quiet_NaN();
    }

   private:
    static std::size_t index(ControlSource s) noexcept { return static_cast<std::size_t>(s); }
    static bool trustedAge(double age_sec, double tolerance_sec) {
        if (tolerance_sec < 0.0)
            return age_sec >= 0.0;  // 禁用新鲜度但仍需“曾收到”
        return contract::sourceAgeAcceptable(age_sec, tolerance_sec);
    }
    static SourceCandidate toCandidate(const SourceObs& o) {
        SourceCandidate c;
        c.source = o.source;
        c.cmd = o.cmd;
        c.checksum = o.checksum;
        c.present = true;
        c.age_sec = 0.0;
        return c;
    }

    static constexpr std::size_t kMaxSources = 8;
    CommandArbitrator arb_;
    ControlSource winner_ = ControlSource::NONE;
    double valid_since_[kMaxSources]{};
    SourceObs last_valid_[kMaxSources]{};
};

// 接线层唯一合法的 SourceObs 构造入口：把**线上帧字段**（帧头/长度/校验和）原样带入，
// 由本工厂统一完成 verifyFrame 与 present 判定。调用方不得自行重算 checksum（见 SourceObs 注）。
[[nodiscard]] inline CommandArbiterFilter::SourceObs makeWireSourceObs(ControlSource source,
                                                                       const VehicleCommandRaw& raw, std::uint8_t head1,
                                                                       std::uint8_t head2, std::uint8_t length,
                                                                       std::uint16_t wire_checksum, double rx_sec) {
    CommandArbiterFilter::SourceObs o;
    o.source = source;
    o.cmd = raw;
    o.checksum = wire_checksum;
    o.last_rx_sec = rx_sec;
    o.present = (rx_sec >= 0.0);
    o.frame_valid = verifyFrame(head1, head2, length);
    return o;
}

inline const char* to_string(CommandArbiterFilter::Untrusted u) noexcept {
    using U = CommandArbiterFilter::Untrusted;
    switch (u) {
        case U::kNone:
            return "trusted";
        case U::kAbsent:
            return "absent";
        case U::kBadFrame:
            return "bad_frame";
        case U::kBadChecksum:
            return "bad_checksum";
        case U::kStale:
            return "stale";
    }
    return "unknown";
}

inline const char* to_string(ControlSource s) noexcept {
    switch (s) {
        case ControlSource::NONE:
            return "none";
        case ControlSource::PURE_PURSUIT:
            return "pure_pursuit";
        case ControlSource::MPC:
            return "mpc";
    }
    return "unknown";
}

}  // namespace vehicle
}  // namespace common_msgs
