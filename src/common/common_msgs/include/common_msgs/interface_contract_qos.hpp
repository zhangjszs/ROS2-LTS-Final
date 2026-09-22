#pragma once
// issue #14：把契约里的 QoS 描述符（interface_contract.h 的 QosSpec，纯 std）落到
// rclcpp::QoS 的唯一构造点。仅由链接 rclcpp 的节点 include（头文件内联，common_msgs 自身不编译）。
#include <cstddef>
#include <rclcpp/qos.hpp>

#include "interface_contract.h"

namespace common_msgs {
namespace contract {

// 由 QosSpec 构造 rclcpp::QoS，使发布/订阅两端共用同一来源，杜绝各处散落构造漂移。
[[nodiscard]] inline rclcpp::QoS makeQoS(const QosSpec& s) {
    rclcpp::QoS qos(rclcpp::KeepLast(s.depth));
    if (s.reliable)
        qos.reliable();
    else
        qos.best_effort();
    // durability 默认即 VOLATILE；仅锁存话题显式改 transient_local，避免调用不存在的 setter。
    if (s.transient_local)
        qos.transient_local();
    return qos;
}

}  // namespace contract
}  // namespace common_msgs
