#include "safety_monitor/stop_state_machine.h"

StopStateMachine::StopStateMachine() : state_(StopState::IDLE), reason_(StopReason::NONE) {}

StopState StopStateMachine::state() const {
    return state_;
}

StopReason StopStateMachine::reason() const {
    return reason_;
}

StopStateMachineResult StopStateMachine::onPathReceived() {
    if (state_ == StopState::TIMEOUT_STOP) {
        state_ = StopState::IDLE;
        reason_ = StopReason::NONE;
        return {StopAction::CLEAR_STOP, StopReason::NONE};
    }
    return {StopAction::NONE, StopReason::NONE};
}

StopStateMachineResult StopStateMachine::onTimeoutExpired() {
    if (state_ == StopState::IDLE) {
        state_ = StopState::TIMEOUT_STOP;
        reason_ = StopReason::TIMEOUT;
        return {StopAction::PUBLISH_STOP, StopReason::TIMEOUT};
    }
    return {StopAction::NONE, StopReason::NONE};
}

StopStateMachineResult StopStateMachine::onStopRequested() {
    // REQUEST_STOP 优先于 TIMEOUT_STOP：完赛/人工停车不能被后续路径清掉。
    if (state_ == StopState::IDLE || state_ == StopState::TIMEOUT_STOP) {
        const bool already_stopped = (state_ == StopState::TIMEOUT_STOP);
        state_ = StopState::REQUEST_STOP;
        reason_ = StopReason::REQUEST;
        if (already_stopped) {
            return {StopAction::NONE, StopReason::REQUEST};
        }
        return {StopAction::PUBLISH_STOP, StopReason::REQUEST};
    }
    return {StopAction::NONE, StopReason::NONE};
}

StopStateMachineResult StopStateMachine::onManualReset() {
    if (state_ == StopState::REQUEST_STOP) {
        state_ = StopState::IDLE;
        reason_ = StopReason::NONE;
        return {StopAction::CLEAR_STOP, StopReason::NONE};
    }
    return {StopAction::NONE, StopReason::NONE};
}
