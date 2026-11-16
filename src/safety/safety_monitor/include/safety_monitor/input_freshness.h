#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include "freshness_lease.h"

namespace safety_monitor {
using common_msgs::vehicle::Freshness;
using common_msgs::vehicle::FreshnessLease;

inline bool validVehicleState(double x, double y, double theta, double speed) {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(theta) && std::isfinite(speed);
}

struct InputHealth {
    Freshness vehicle;
    Freshness planner;
    double vehicle_age;
    double planner_age;
    bool healthy() const { return vehicle == Freshness::kFresh && planner == Freshness::kFresh; }
};

inline const char* freshnessName(Freshness state) {
    switch (state) {
        case Freshness::kAbsent:
            return "Absent";
        case Freshness::kFresh:
            return "Fresh";
        case Freshness::kStale:
            return "Stale";
        case Freshness::kFromFuture:
            return "FromFuture";
    }
    return "Unknown";
}

// #107：独立接收租约，rx 由 steady_clock 提供，ros 只用于检测时间回退。
// 不使用 HuatCarstate.header（发布时刻）或未经标定的设备 GPS 时间推算测量年龄。
// 回调/看门狗必须由适配层用同一互斥锁串行调用。所有时间均可由测试注入。
class InputFreshness {
   public:
    explicit InputFreshness(double state_timeout = 0.3, double path_timeout = 0.5)
        : vehicle_({checkedTimeout(state_timeout), 0.0}), planner_({checkedTimeout(path_timeout), 0.0}) {}

    void observeVehicle(bool valid, double rx, double ros) {
        if (syncClock(rx, ros) && valid)
            vehicle_.observe(rx);
    }
    void observePath(bool valid, double rx, double ros) {
        if (syncClock(rx, ros) && valid)
            planner_.observe(rx);
    }
    InputHealth check(double rx, double ros) {
        if (!syncClock(rx, ros))
            return {Freshness::kAbsent, Freshness::kAbsent, infinity(), infinity()};
        return {vehicle_.check(rx), planner_.check(rx), vehicle_.age(rx), planner_.age(rx)};
    }
    bool healthy(double rx, double ros) { return check(rx, ros).healthy(); }

   private:
    static double infinity() { return std::numeric_limits<double>::infinity(); }
    static double checkedTimeout(double timeout) {
        if (!std::isfinite(timeout) || timeout <= 0.0)
            throw std::invalid_argument("safety input timeout must be finite and positive");
        return timeout;
    }
    bool syncClock(double rx, double ros) {
        if (!std::isfinite(rx) || !std::isfinite(ros)) {
            invalidate();
            last_rx_ = last_ros_ = std::numeric_limits<double>::quiet_NaN();
            return false;
        }
        // FromFuture（接收时钟回退）或 ROS 回退：旧 epoch 的两份租约一并作废；
        // 回退后的当前合法帧只续自己的租约，必须重新收到另一个源才能恢复。
        if ((std::isfinite(last_rx_) && rx < last_rx_) || (std::isfinite(last_ros_) && ros < last_ros_))
            invalidate();
        last_rx_ = rx;
        last_ros_ = ros;
        return true;
    }
    void invalidate() {
        vehicle_.observe(std::numeric_limits<double>::quiet_NaN());
        planner_.observe(std::numeric_limits<double>::quiet_NaN());
    }
    FreshnessLease vehicle_;
    FreshnessLease planner_;
    double last_rx_ = std::numeric_limits<double>::quiet_NaN();
    double last_ros_ = std::numeric_limits<double>::quiet_NaN();
};
}  // namespace safety_monitor
