#pragma once

#include "freshness_lease.h"

namespace safety_monitor {
// #107：先抽离现有接收活性判定，保持基线语义以建立 RED；修复在后续提交。
class InputFreshness {
   public:
    explicit InputFreshness(double state_timeout = 0.3, double path_timeout = 0.5)
        : planner_({path_timeout, 0.0}) { (void)state_timeout; }

    void observeVehicle(bool valid, double rx, double /*ros*/) {
        if (!valid)
            return;
        if (!state_seen_)
            planner_.observe(rx);
        state_seen_ = true;
    }
    void observePath(bool valid, double rx, double /*ros*/) {
        if (valid)
            planner_.observe(rx);
    }
    bool healthy(double rx, double /*ros*/) { return state_seen_ && planner_.check(rx) == common_msgs::vehicle::Freshness::kFresh; }

   private:
    bool state_seen_ = false;
    common_msgs::vehicle::FreshnessLease planner_;
};
}  // namespace safety_monitor
