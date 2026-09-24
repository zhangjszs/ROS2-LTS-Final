#pragma once
// ============================================================================
// issue #16：统一“任务状态 × 安全状态”状态机（纯 std，无 ROS context，可单测）。
//
// 职责边界（避免重复实现）：
//   - safety_monitor 的 StopStateMachine 负责“看门狗触发”：超时/请求/停车的时机判定（#7/#8）。
//   - 本头负责“系统级编排词汇”：把任务生命周期（idle/armed/running/finished/fault）与安全状态
//     （normal/degraded/stop）统一成显式、可测的状态迁移，并保证与安全 #8 的锁存语义一致：
//       · TIMEOUT 停：可被恢复(Resume)清除；
//       · REQUEST/FAULT 停：锁存(sticky)，仅人工 Reset 清除，路径恢复不得解除。
//   仲裁层（command_arbitration.h）据此判断是否强制安全降级。
//
// 与契约层衔接：StopKind 语义与 StopStateMachine 的 StopReason 对齐，便于 safety_monitor 直接喂入。
// ============================================================================
#include <string_view>

namespace common_msgs {
namespace vehicle {

enum class TaskState { IDLE, ARMED, RUNNING, FINISHED, FAULT };

enum class SafetyState { NORMAL, DEGRADED, STOP };

// 停车触发来源：与 #8 锁存优先级一致（REQUEST/FAULT 锁存 > TIMEOUT 可恢复）。
enum class StopKind { NONE, TIMEOUT, REQUEST, FAULT };

// 状态迁移产生的“停车发布边沿”，映射到 HuatStop 的锁存发布（true=停车, false=清除）。
enum class StopEdge { NONE, PUBLISH, CLEAR };

struct TaskSafetyTransition {
    bool changed;
    StopEdge stop_edge;
};

constexpr std::string_view to_string_view(TaskState s) noexcept {
    switch (s) {
        case TaskState::IDLE:
            return "IDLE";
        case TaskState::ARMED:
            return "ARMED";
        case TaskState::RUNNING:
            return "RUNNING";
        case TaskState::FINISHED:
            return "FINISHED";
        case TaskState::FAULT:
            return "FAULT";
    }
    return "UNKNOWN";
}

constexpr std::string_view to_string_view(SafetyState s) noexcept {
    switch (s) {
        case SafetyState::NORMAL:
            return "NORMAL";
        case SafetyState::DEGRADED:
            return "DEGRADED";
        case SafetyState::STOP:
            return "STOP";
    }
    return "UNKNOWN";
}

constexpr std::string_view to_string_view(StopKind k) noexcept {
    switch (k) {
        case StopKind::NONE:
            return "none";
        case StopKind::TIMEOUT:
            return "timeout";
        case StopKind::REQUEST:
            return "request";
        case StopKind::FAULT:
            return "fault";
    }
    return "unknown";
}

// 任务 × 安全 统一状态机。所有迁移显式、幂等（重复事件不再产生命令边沿），可单测。
class TaskSafetyStateMachine {
   public:
    TaskState task() const noexcept { return task_; }
    SafetyState safety() const noexcept { return safety_; }
    StopKind stopKind() const noexcept { return stop_kind_; }
    // 安全是否处于“禁止行驶”：STOP 或任务已 FAULT/FINISHED（两者均触发锁存停）。
    bool stopActive() const noexcept { return safety_ == SafetyState::STOP; }
    // 允许输出行驶指令：仅当非停 且 任务处于 RUNNING（ARMED 为已就绪未起步，仍按安全制动处理）。
    bool canDrive() const noexcept { return safety_ != SafetyState::STOP && task_ == TaskState::RUNNING; }

    // —— 安全事件 ——
    // 超时停车：可被 Resume 清除；若已在锁存停(REQUEST/FAULT)则忽略（锁存优先）。
    TaskSafetyTransition onTimeoutStop() { return enterStop(StopKind::TIMEOUT); }
    // 请求/完赛停车：锁存，仅 Reset 清除。
    TaskSafetyTransition onStopRequest() { return enterStop(StopKind::REQUEST); }
    // 故障：锁存停 + 任务进入 FAULT，最高优先级。
    TaskSafetyTransition onFault() {
        auto t = enterStop(StopKind::FAULT);
        if (task_ != TaskState::FAULT) {
            task_ = TaskState::FAULT;
            t.changed = true;
        }
        return t;
    }
    // 恢复：仅清除 TIMEOUT 停；锁存停不动。
    TaskSafetyTransition onResume() {
        if (safety_ == SafetyState::STOP && stop_kind_ == StopKind::TIMEOUT) {
            safety_ = SafetyState::NORMAL;
            stop_kind_ = StopKind::NONE;
            return {true, StopEdge::CLEAR};
        }
        return {false, StopEdge::NONE};
    }
    // 人工复位：清除任意锁存停；FAULT/FINISHED 任务回到 IDLE。
    TaskSafetyTransition onReset() {
        bool changed = false;
        StopEdge edge = StopEdge::NONE;
        if (safety_ == SafetyState::STOP) {
            safety_ = SafetyState::NORMAL;
            stop_kind_ = StopKind::NONE;
            edge = StopEdge::CLEAR;
            changed = true;
        }
        if (task_ == TaskState::FAULT || task_ == TaskState::FINISHED) {
            task_ = TaskState::IDLE;
            changed = true;
        }
        return {changed, edge};
    }

    // —— 任务事件（安全 STOP 时一律拒绝，除非由 FINISH 主动进入停）——
    TaskSafetyTransition onArm() {
        if (task_ == TaskState::IDLE && safety_ != SafetyState::STOP) {
            task_ = TaskState::ARMED;
            return {true, StopEdge::NONE};
        }
        return {false, StopEdge::NONE};
    }
    TaskSafetyTransition onStart() {
        if ((task_ == TaskState::ARMED || task_ == TaskState::RUNNING) && safety_ != SafetyState::STOP) {
            if (task_ != TaskState::RUNNING) {
                task_ = TaskState::RUNNING;
                return {true, StopEdge::NONE};
            }
        }
        return {false, StopEdge::NONE};
    }
    // 完赛：任务 FINISHED 并触发锁存停（对齐 #8“完赛停车不被后续路径清掉”）。
    TaskSafetyTransition onFinish() {
        const bool was_finished = (task_ == TaskState::FINISHED);
        auto t = enterStop(StopKind::REQUEST);
        task_ = TaskState::FINISHED;
        t.changed = t.changed || !was_finished;
        return t;
    }

   private:
    TaskSafetyTransition enterStop(StopKind kind) {
        // 锁存优先级：已在锁存停(REQUEST/FAULT)时，非更高强度的 TIMEOUT 不覆盖；相同来源幂等。
        if (safety_ == SafetyState::STOP && stop_kind_ != StopKind::TIMEOUT && kind == StopKind::TIMEOUT) {
            return {false, StopEdge::NONE};
        }
        const bool first_edge = (safety_ != SafetyState::STOP);
        safety_ = SafetyState::STOP;
        stop_kind_ = kind;
        return {first_edge, first_edge ? StopEdge::PUBLISH : StopEdge::NONE};
    }

    TaskState task_ = TaskState::IDLE;
    SafetyState safety_ = SafetyState::NORMAL;
    StopKind stop_kind_ = StopKind::NONE;
};

}  // namespace vehicle
}  // namespace common_msgs
