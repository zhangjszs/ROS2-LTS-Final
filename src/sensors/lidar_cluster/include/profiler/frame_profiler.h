#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <chrono>

/**
 * @brief 帧级计时分析器，带自动告警功能。
 *
 * 发布每帧的耗时分解，并在处理时间超过可配置阈值时发出 WARN/ERROR 日志。
 *
 * 话题布局 (Float64MultiArray):
 *   [0] PassThrough 耗时 (ms)
 *   [1] 地面分割耗时 (ms)
 *   [2] 聚类耗时 (ms)
 *   [3] 总帧耗时 (ms)
 *   [4] 无数据帧计数（启动时没有点云的帧）
 */
class FrameProfiler {
   public:
    explicit FrameProfiler(rclcpp::Node* node, const std::string &topic, double warn_ms, double error_ms,
                           int consecutive_threshold);
    explicit FrameProfiler(rclcpp::Node& node, const std::string &topic, double warn_ms, double error_ms,
                           int consecutive_threshold)
        : FrameProfiler(&node, topic, warn_ms, error_ms, consecutive_threshold) {}
    explicit FrameProfiler(const rclcpp::Node::SharedPtr& node, const std::string &topic, double warn_ms, double error_ms,
                           int consecutive_threshold)
        : FrameProfiler(node.get(), topic, warn_ms, error_ms, consecutive_threshold) {}

    /**
     * @brief 发布性能分析数据并评估自动告警阈值。
     */
    void publish(double pt_ms, double seg_ms, double cluster_ms, double total_ms, int no_data_frames);

    /**
     * @brief 格式化单帧耗时统计摘要 (C++20 std::format)
     */
    static std::string formatSummary(double pt_ms, double seg_ms, double cluster_ms, double total_ms, int no_data_frames);

    /**
     * @brief 格式化超时警告信息 (C++20 std::format)
     */
    static std::string formatWarnMessage(double total_ms, double warn_ms, int consecutive, int no_data_frames = -1);

    /**
     * @brief 格式化超时严重错误信息 (C++20 std::format)
     */
    static std::string formatErrorMessage(double total_ms, double error_ms);

    /**
     * @brief 格式化恢复正常信息 (C++20 std::format)
     */
    static std::string formatRecoveryMessage(double total_ms, int was_consecutive);

   private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
    double warn_ms_;
    double error_ms_;
    int consecutive_threshold_;
    int consecutive_timeout_count_ = 0;
    rclcpp::Clock clock_;
};
