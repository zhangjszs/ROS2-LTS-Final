#include <gtest/gtest.h>

#include "safety_monitor/stop_state_machine.h"

// ── Initial state ─────────────────────────────────────────────────────────

TEST(StopStateMachineTest, InitialStateIsIdle) {
    StopStateMachine fsm;
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
}

// ── Timeout flow: IDLE → TIMEOUT_STOP → IDLE ──────────────────────────────

TEST(StopStateMachineTest, TimeoutFromIdlePublishesStop) {
    StopStateMachine fsm;
    auto result = fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::TIMEOUT);
    EXPECT_EQ(result.action, StopAction::PUBLISH_STOP);
    EXPECT_EQ(result.reason, StopReason::TIMEOUT);
}

TEST(StopStateMachineTest, PathReceivedClearsTimeoutStop) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
    EXPECT_EQ(result.action, StopAction::CLEAR_STOP);
    EXPECT_EQ(result.reason, StopReason::NONE);
}

// ── Request flow: IDLE → REQUEST_STOP → IDLE ──────────────────────────────

TEST(StopStateMachineTest, StopRequestFromIdlePublishesStop) {
    StopStateMachine fsm;
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::REQUEST);
    EXPECT_EQ(result.action, StopAction::PUBLISH_STOP);
    EXPECT_EQ(result.reason, StopReason::REQUEST);
}

TEST(StopStateMachineTest, ManualResetClearsRequestStop) {
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(fsm.reason(), StopReason::NONE);
    EXPECT_EQ(result.action, StopAction::CLEAR_STOP);
    EXPECT_EQ(result.reason, StopReason::NONE);
}

// ── Guard: no-op on invalid transitions ────────────────────────────────────

TEST(StopStateMachineTest, TimeoutWhileAlreadyInTimeoutIsNoOp) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, StopRequestWhileAlreadyInRequestIsNoOp) {
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, PathReceivedWhileIdleIsNoOp) {
    StopStateMachine fsm;
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, ManualResetWhileIdleIsNoOp) {
    StopStateMachine fsm;
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Guard: manual reset only works from REQUEST_STOP ───────────────────────

TEST(StopStateMachineTest, ManualResetFromTimeoutStopIsNoOp) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Recovery: path received during REQUEST_STOP does NOT clear ─────────────

TEST(StopStateMachineTest, PathReceivedDuringRequestStopIsNoOp) {
    // REQUEST_STOP requires explicit manual reset, path alone won't clear it
    StopStateMachine fsm;
    fsm.onStopRequested();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, StopRequestDuringTimeoutPromotesToRequestStop) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    auto result = fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(fsm.reason(), StopReason::REQUEST);
    EXPECT_EQ(result.action, StopAction::NONE);
}

TEST(StopStateMachineTest, PathReceivedAfterTimeoutThenRequestDoesNotResume) {
    StopStateMachine fsm;
    fsm.onTimeoutExpired();
    fsm.onStopRequested();
    auto result = fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    EXPECT_EQ(result.action, StopAction::NONE);
}

// ── Full cycle: timeout → recover → request → reset ────────────────────────

TEST(StopStateMachineTest, FullCycle) {
    StopStateMachine fsm;
    // timeout triggers stop
    fsm.onTimeoutExpired();
    EXPECT_EQ(fsm.state(), StopState::TIMEOUT_STOP);
    // planner recovers
    fsm.onPathReceived();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
    // race finish triggers request stop
    fsm.onStopRequested();
    EXPECT_EQ(fsm.state(), StopState::REQUEST_STOP);
    // manual reset clears
    fsm.onManualReset();
    EXPECT_EQ(fsm.state(), StopState::IDLE);
}

// ── C++20 std::format tests ───────────────────────────────────────────────

TEST(StopStateMachineFormatTest, FormatsStopStateCorrectly) {
    EXPECT_EQ(std::format("{}", StopState::IDLE), "IDLE");
    EXPECT_EQ(std::format("{}", StopState::TIMEOUT_STOP), "TIMEOUT_STOP");
    EXPECT_EQ(std::format("{}", StopState::REQUEST_STOP), "REQUEST_STOP");
    EXPECT_EQ(std::format("{:>15}", StopState::IDLE), "           IDLE");
}

TEST(StopStateMachineFormatTest, FormatsStopReasonCorrectly) {
    EXPECT_EQ(std::format("{}", StopReason::NONE), "none");
    EXPECT_EQ(std::format("{}", StopReason::TIMEOUT), "timeout");
    EXPECT_EQ(std::format("{}", StopReason::REQUEST), "request");
}

TEST(StopStateMachineFormatTest, FormatsStopActionCorrectly) {
    EXPECT_EQ(std::format("{}", StopAction::NONE), "NONE");
    EXPECT_EQ(std::format("{}", StopAction::PUBLISH_STOP), "PUBLISH_STOP");
    EXPECT_EQ(std::format("{}", StopAction::CLEAR_STOP), "CLEAR_STOP");
}

TEST(StopStateMachineFormatTest, FormatsCompositeDiagnosticString) {
    StopState state = StopState::TIMEOUT_STOP;
    StopReason reason = StopReason::TIMEOUT;
    std::string diag = std::format("State is {}, Reason is {}", state, reason);
    EXPECT_EQ(diag, "State is TIMEOUT_STOP, Reason is timeout");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
