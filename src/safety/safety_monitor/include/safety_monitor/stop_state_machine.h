#ifndef SAFETY_MONITOR_STOP_STATE_MACHINE_H
#define SAFETY_MONITOR_STOP_STATE_MACHINE_H

#include <format>
#include <string_view>

enum class StopState { IDLE, TIMEOUT_STOP, REQUEST_STOP };

enum class StopReason { NONE, TIMEOUT, REQUEST };

enum class StopAction { NONE, PUBLISH_STOP, CLEAR_STOP };

constexpr std::string_view to_string_view(StopState state) noexcept {
    switch (state) {
        case StopState::IDLE:
            return "IDLE";
        case StopState::TIMEOUT_STOP:
            return "TIMEOUT_STOP";
        case StopState::REQUEST_STOP:
            return "REQUEST_STOP";
    }
    return "UNKNOWN";
}

constexpr std::string_view to_string_view(StopReason reason) noexcept {
    switch (reason) {
        case StopReason::NONE:
            return "none";
        case StopReason::TIMEOUT:
            return "timeout";
        case StopReason::REQUEST:
            return "request";
    }
    return "unknown";
}

constexpr std::string_view to_string_view(StopAction action) noexcept {
    switch (action) {
        case StopAction::NONE:
            return "NONE";
        case StopAction::PUBLISH_STOP:
            return "PUBLISH_STOP";
        case StopAction::CLEAR_STOP:
            return "CLEAR_STOP";
    }
    return "UNKNOWN";
}

struct StopStateMachineResult {
    StopAction action;
    StopReason reason;
};

class StopStateMachine {
   public:
    StopStateMachine();

    StopState state() const;
    StopReason reason() const;

    StopStateMachineResult onPathReceived();
    StopStateMachineResult onTimeoutExpired();
    StopStateMachineResult onStopRequested();
    StopStateMachineResult onManualReset();

   private:
    StopState state_;
    StopReason reason_;
};

// ── C++20 std::formatter 特化：使状态机类型支持原生 std::format ──────────

template <>
struct std::formatter<StopState> : std::formatter<std::string_view> {
    template <typename FormatContext>
    auto format(StopState state, FormatContext& ctx) const {
        return std::formatter<std::string_view>::format(to_string_view(state), ctx);
    }
};

template <>
struct std::formatter<StopReason> : std::formatter<std::string_view> {
    template <typename FormatContext>
    auto format(StopReason reason, FormatContext& ctx) const {
        return std::formatter<std::string_view>::format(to_string_view(reason), ctx);
    }
};

template <>
struct std::formatter<StopAction> : std::formatter<std::string_view> {
    template <typename FormatContext>
    auto format(StopAction action, FormatContext& ctx) const {
        return std::formatter<std::string_view>::format(to_string_view(action), ctx);
    }
};

#endif
