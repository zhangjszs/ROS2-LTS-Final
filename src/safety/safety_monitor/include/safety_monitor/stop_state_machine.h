#ifndef SAFETY_MONITOR_STOP_STATE_MACHINE_H
#define SAFETY_MONITOR_STOP_STATE_MACHINE_H

enum class StopState { IDLE, TIMEOUT_STOP, REQUEST_STOP };

enum class StopReason { NONE, TIMEOUT, REQUEST };

enum class StopAction { NONE, PUBLISH_STOP, CLEAR_STOP };

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

#endif
