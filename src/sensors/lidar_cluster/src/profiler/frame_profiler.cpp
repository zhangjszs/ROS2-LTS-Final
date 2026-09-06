#include "profiler/frame_profiler.h"

#include <format>

FrameProfiler::FrameProfiler(rclcpp::Node::SharedPtr node, const std::string &topic, double warn_ms, double error_ms,
                             int consecutive_threshold)
    : warn_ms_(warn_ms), error_ms_(error_ms), consecutive_threshold_(consecutive_threshold) {
    pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic, 10);
}

std::string FrameProfiler::formatSummary(double pt_ms, double seg_ms, double cluster_ms, double total_ms,
                                         int no_data_frames) {
    return std::format(
        "[lidar_cluster] Profiling (ms) | PassThrough={:.2f} GroundSeg={:.2f} Cluster={:.2f} Total={:.2f} "
        "NoData={}",
        pt_ms, seg_ms, cluster_ms, total_ms, no_data_frames);
}

std::string FrameProfiler::formatWarnMessage(double total_ms, double warn_ms, int consecutive, int no_data_frames) {
    if (no_data_frames >= 0) {
        return std::format(
            "[lidar_cluster] Frame time {:.2f} ms exceeds WARN threshold {:.2f} ms "
            "(consecutive={}, no_data={})",
            total_ms, warn_ms, consecutive, no_data_frames);
    }
    return std::format(
        "[lidar_cluster] Frame time {:.2f} ms exceeds WARN threshold {:.2f} ms (consecutive={})",
        total_ms, warn_ms, consecutive);
}

std::string FrameProfiler::formatErrorMessage(double total_ms, double error_ms) {
    return std::format("[lidar_cluster] Frame time {:.2f} ms exceeds ERROR threshold {:.2f} ms", total_ms, error_ms);
}

std::string FrameProfiler::formatRecoveryMessage(double total_ms, int was_consecutive) {
    return std::format("[lidar_cluster] Frame time recovered to {:.2f} ms (was consecutive={})", total_ms,
                       was_consecutive);
}

void FrameProfiler::publish(double pt_ms, double seg_ms, double cluster_ms, double total_ms, int no_data_frames) {
    std_msgs::msg::Float64MultiArray arr;
    arr.data = {pt_ms, seg_ms, cluster_ms, total_ms, static_cast<double>(no_data_frames)};
    pub_->publish(arr);

    auto logger = rclcpp::get_logger("lidar_cluster");

    if (total_ms > error_ms_) {
        RCLCPP_ERROR(logger, "%s", formatErrorMessage(total_ms, error_ms_).c_str());
    } else if (total_ms > warn_ms_) {
        consecutive_timeout_count_++;
        if (consecutive_timeout_count_ >= consecutive_threshold_) {
            RCLCPP_WARN_THROTTLE(logger, clock_, 1000, "%s",
                              formatWarnMessage(total_ms, warn_ms_, consecutive_timeout_count_, no_data_frames).c_str());
        } else {
            RCLCPP_WARN_THROTTLE(logger, clock_, 1000, "%s",
                              formatWarnMessage(total_ms, warn_ms_, consecutive_timeout_count_).c_str());
        }
    } else {
        if (consecutive_timeout_count_ > 0) {
            RCLCPP_INFO(logger, "%s", formatRecoveryMessage(total_ms, consecutive_timeout_count_).c_str());
        }
        consecutive_timeout_count_ = 0;
    }

    RCLCPP_INFO_THROTTLE(logger, clock_, 5000, "%s",
                      formatSummary(pt_ms, seg_ms, cluster_ms, total_ms, no_data_frames).c_str());
}
