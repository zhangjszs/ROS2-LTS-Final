#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <preprocessor/point_cloud_preprocessor.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include "string_utils.h"

static void CompactPoints(pcl::PointCloud<PointType>::Ptr cloud) {
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
}

// Keep 1m..15m in front. In-place compact instead of ExtractIndices (extra copy).
static void ClipPointsByDistance(const pcl::PointCloud<PointType>::Ptr in) {
    auto& pts = in->points;
    std::erase_if(pts, [](const PointType& p) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x <= 0.0f)
            return true;
        const float d2 = p.x * p.x + p.y * p.y;
        return d2 < 1.0f || d2 > 225.0f;
    });
    CompactPoints(in);
}

static void FilterCloudAabb(pcl::PointCloud<PointType>::Ptr cloud, const PointCloudPreprocessor::RoiBounds& roi) {
    const float x_min = static_cast<float>(roi.x_min);
    const float x_max = static_cast<float>(roi.x_max);
    const float y_min = static_cast<float>(roi.y_min);
    const float y_max = static_cast<float>(roi.y_max);
    const float z_min = static_cast<float>(roi.z_min);
    const float z_max = static_cast<float>(roi.z_max);
    auto& pts = cloud->points;
    std::erase_if(pts, [&](const PointType& p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || p.x < x_min || p.x > x_max ||
               p.y < y_min || p.y > y_max || p.z < z_min || p.z > z_max;
    });
    CompactPoints(cloud);
}

std::vector<PointCloudPreprocessor::PointSpan> PointCloudPreprocessor::slicePointCloud(PointSpan points,
                                                                                       size_t num_segments) {
    std::vector<PointSpan> slices;
    if (points.empty() || num_segments == 0) {
        return slices;
    }
    slices.reserve(num_segments);
    const size_t total = points.size();
    const size_t base_chunk = total / num_segments;
    const size_t remainder = total % num_segments;
    size_t offset = 0;

    for (size_t i = 0; i < num_segments; ++i) {
        const size_t chunk_len = base_chunk + (i < remainder ? 1 : 0);
        if (chunk_len > 0 && offset < total) {
            slices.push_back(points.subspan(offset, chunk_len));
            offset += chunk_len;
        }
    }
    return slices;
}

size_t PointCloudPreprocessor::countValidPointsInRoi(PointSpan points, const RoiBounds& roi) {
    const float x_min = static_cast<float>(roi.x_min);
    const float x_max = static_cast<float>(roi.x_max);
    const float y_min = static_cast<float>(roi.y_min);
    const float y_max = static_cast<float>(roi.y_max);
    const float z_min = static_cast<float>(roi.z_min);
    const float z_max = static_cast<float>(roi.z_max);

    return static_cast<size_t>(std::ranges::count_if(points, [&](const PointType& p) {
        return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && p.x >= x_min && p.x <= x_max &&
               p.y >= y_min && p.y <= y_max && p.z >= z_min && p.z <= z_max;
    }));
}

template <typename T>
static void SafeDeclareParam(rclcpp::Node* node, const std::string& name, const T& default_val) {
    if (!node->has_parameter(name)) {
        node->declare_parameter(name, default_val);
    }
}

void PointCloudPreprocessor::LoadZParams(rclcpp::Node* node) {
    SafeDeclareParam(node, "road_type", 2);
    SafeDeclareParam(node, "z_up", 0.7);
    SafeDeclareParam(node, "z_down", -1.0);

    node->get_parameter("road_type", road_type_);
    node->get_parameter("z_up", z_up_);
    node->get_parameter("z_down", z_down_);
}

void PointCloudPreprocessor::LoadROIParams(rclcpp::Node* node) {
    SafeDeclareParam(node, "accel_x_max", 0.0);
    SafeDeclareParam(node, "accel_x_min", 0.0);
    SafeDeclareParam(node, "accel_y_max", 0.0);
    SafeDeclareParam(node, "accel_y_min", 0.0);
    SafeDeclareParam(node, "track_x_max", 0.0);
    SafeDeclareParam(node, "track_x_min", 0.0);
    SafeDeclareParam(node, "track_y_max", 0.0);
    SafeDeclareParam(node, "track_y_min", 0.0);
    SafeDeclareParam(node, "enable_dynamic_roi", false);
    SafeDeclareParam(node, "pitch_compensation", true);
    SafeDeclareParam(node, "speed_compensation", true);
    SafeDeclareParam(node, "z_pitch_scale", 0.1);
    SafeDeclareParam(node, "speed_x_scale", 0.5);
    SafeDeclareParam(node, "speed_y_shrink", 0.1);
    SafeDeclareParam(node, "min_roi_x_max", 20.0);
    SafeDeclareParam(node, "max_roi_x_max", 60.0);

    node->get_parameter("accel_x_max", accel_x_max_);
    node->get_parameter("accel_x_min", accel_x_min_);
    node->get_parameter("accel_y_max", accel_y_max_);
    node->get_parameter("accel_y_min", accel_y_min_);
    node->get_parameter("track_x_max", track_x_max_);
    node->get_parameter("track_x_min", track_x_min_);
    node->get_parameter("track_y_max", track_y_max_);
    node->get_parameter("track_y_min", track_y_min_);
    node->get_parameter("enable_dynamic_roi", enable_dynamic_roi_);
    node->get_parameter("pitch_compensation", pitch_compensation_);
    node->get_parameter("speed_compensation", speed_compensation_);
    node->get_parameter("z_pitch_scale", z_pitch_scale_);
    node->get_parameter("speed_x_scale", speed_x_scale_);
    node->get_parameter("speed_y_shrink", speed_y_shrink_);
    node->get_parameter("min_roi_x_max", min_roi_x_max_);
    node->get_parameter("max_roi_x_max", max_roi_x_max_);
}

void PointCloudPreprocessor::LoadVoxelParams(rclcpp::Node* node) {
    SafeDeclareParam(node, "enable_adaptive_voxel", false);
    SafeDeclareParam(node, "voxel_ranges", std::string("5,15"));
    SafeDeclareParam(node, "voxel_leaf_sizes", std::string("0.03,0.05,0.10"));
    SafeDeclareParam(node, "enable_sor", false);
    SafeDeclareParam(node, "sor_mean_k", 10);
    SafeDeclareParam(node, "sor_stddev", 1.0);
    SafeDeclareParam(node, "frp_coarse_leaf_size", 0.08);

    node->get_parameter("enable_adaptive_voxel", enable_adaptive_voxel_);
    node->get_parameter("voxel_ranges", voxel_ranges_str_);
    node->get_parameter("voxel_leaf_sizes", voxel_leaf_sizes_str_);
    node->get_parameter("enable_sor", enable_sor_);
    node->get_parameter("sor_mean_k", sor_mean_k_);
    node->get_parameter("sor_stddev", sor_stddev_);
    node->get_parameter("frp_coarse_leaf_size", frp_coarse_leaf_size_);
}

PointCloudPreprocessor::PointCloudPreprocessor(rclcpp::Node* node) {
    LoadZParams(node);
    LoadROIParams(node);
    LoadVoxelParams(node);

    // 预解析自适应体素参数，避免每帧重复字符串分割
    if (enable_adaptive_voxel_) {
        splitString(voxel_ranges_str_, voxel_ranges_);
        splitString(voxel_leaf_sizes_str_, voxel_leaf_sizes_);
        if (voxel_ranges_.empty() || voxel_leaf_sizes_.empty() ||
            voxel_leaf_sizes_.size() != voxel_ranges_.size() + 1) {
            RCLCPP_WARN(rclcpp::get_logger("lidar_cluster"),
                        "[lidar_cluster] Invalid adaptive voxel config, adaptive voxel disabled");
            enable_adaptive_voxel_ = false;
        }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("lidar_cluster"),
        "[lidar_cluster/preprocessor] road_type=%d, z_up=%.2f, z_down=%.2f, adaptive_voxel=%s, sor=%s, dynamic_roi=%s",
        road_type_, z_up_, z_down_, enable_adaptive_voxel_ ? "true" : "false", enable_sor_ ? "true" : "false",
        enable_dynamic_roi_ ? "true" : "false");
}

void PointCloudPreprocessor::splitString(const std::string& in_string, std::vector<double>& out_array) {
    lidar_cluster::parseCsvDoubles(in_string, out_array);
}

void PointCloudPreprocessor::GetBaseXYBounds(double& x_min, double& x_max, double& y_min, double& y_max) const {
    if (road_type_ == 2) {
        x_min = accel_x_min_;
        x_max = accel_x_max_;
        y_min = accel_y_min_;
        y_max = accel_y_max_;
    } else if (road_type_ == 3) {
        x_min = track_x_min_;
        x_max = track_x_max_;
        y_min = track_y_min_;
        y_max = track_y_max_;
    } else {
        x_min = x_max = y_min = y_max = 0;
    }
}

PointCloudPreprocessor::RoiBounds PointCloudPreprocessor::computeDynamicROIBounds(double current_pitch,
                                                                                  double current_speed) const {
    RoiBounds b;
    GetBaseXYBounds(b.x_min, b.x_max, b.y_min, b.y_max);

    double pitch_offset = pitch_compensation_ ? current_pitch * z_pitch_scale_ : 0.0;
    b.z_min = z_down_ + pitch_offset;
    b.z_max = z_up_ + pitch_offset;

    if (enable_dynamic_roi_ && speed_compensation_ && current_speed > 0) {
        double x_base = (road_type_ == 2) ? accel_x_max_ : track_x_max_;
        b.x_max = std::min(max_roi_x_max_, x_base + current_speed * speed_x_scale_);
        b.y_min += current_speed * speed_y_shrink_;
        b.y_max -= current_speed * speed_y_shrink_;
    }
    return b;
}

void PointCloudPreprocessor::adaptiveVoxelGrid(pcl::PointCloud<PointType>::Ptr& cloud, bool frp_active) {
    if (cloud->empty())
        return;

    std::span<const double> ranges(voxel_ranges_);
    std::span<const double> leaf_sizes(voxel_leaf_sizes_);
    PointSpan point_span(cloud->points.data(), cloud->points.size());

    std::vector<float> ranges_sq(ranges.size());
    for (size_t i = 0; i < ranges.size(); i++) {
        ranges_sq[i] = static_cast<float>(ranges[i] * ranges[i]);
    }

    size_t num_bins = leaf_sizes.size();
    std::vector<pcl::PointCloud<PointType>::Ptr> bins(num_bins);
    const size_t n = point_span.size();
    for (size_t i = 0; i < num_bins; i++) {
        bins[i].reset(new pcl::PointCloud<PointType>());
        bins[i]->points.reserve(n);
    }

    for (const auto& p : point_span) {
        float d2 = p.x * p.x + p.y * p.y;
        size_t bin = num_bins - 1;
        for (size_t i = 0; i < ranges_sq.size(); i++) {
            if (d2 < ranges_sq[i]) {
                bin = i;
                break;
            }
        }
        bins[bin]->push_back(p);
    }

    pcl::PointCloud<PointType>::Ptr result(new pcl::PointCloud<PointType>());
    size_t total_approx = 0;
    for (const auto& b : bins)
        total_approx += b->size();
    result->points.reserve(total_approx / 2);

    for (size_t i = 0; i < num_bins; i++) {
        if (bins[i]->empty())
            continue;
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(bins[i]);
        float leaf = frp_active ? static_cast<float>(frp_coarse_leaf_size_) : static_cast<float>(leaf_sizes[i]);
        vg.setLeafSize(leaf, leaf, leaf);
        pcl::PointCloud<PointType> tmp;
        vg.filter(tmp);
        if (!tmp.empty()) {
            *result += tmp;
        }
    }
    if (!result->empty()) {
        cloud = result;
    } else {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("lidar_cluster"), clock_, 5000,
                             "[lidar_cluster] Adaptive voxel produced empty cloud, keeping original (%zu pts)",
                             cloud->size());
    }
}

void PointCloudPreprocessor::applySOR(pcl::PointCloud<PointType>::Ptr& cloud) {
    if (cloud->empty() || sor_mean_k_ <= 0)
        return;
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_stddev_);
    sor.filter(*cloud);
}

void PointCloudPreprocessor::process(pcl::PointCloud<PointType>::Ptr& cloud_filtered, bool frp_active,
                                     double current_pitch, double current_speed) {
    RCLCPP_DEBUG(rclcpp::get_logger("lidar_cluster"), "[lidar_cluster] Points before PassThrough: %zu",
                 cloud_filtered->points.size());

    RoiBounds roi = computeDynamicROIBounds(current_pitch, current_speed);

    if (road_type_ == 1) {
        ClipPointsByDistance(cloud_filtered);
        const float z_min = static_cast<float>(roi.z_min);
        const float z_max = static_cast<float>(roi.z_max);
        auto& pts = cloud_filtered->points;
        std::erase_if(pts,
                      [z_min, z_max](const PointType& p) { return !std::isfinite(p.z) || p.z < z_min || p.z > z_max; });
        CompactPoints(cloud_filtered);
    } else if (road_type_ == 2 || road_type_ == 3) {
        FilterCloudAabb(cloud_filtered, roi);
    } else {
        throw std::runtime_error("[lidar_cluster] Undefined road_type: " + std::to_string(road_type_));
    }

    RCLCPP_DEBUG(rclcpp::get_logger("lidar_cluster"), "[lidar_cluster] Points before downsample: %zu",
                 cloud_filtered->points.size());

    if (enable_adaptive_voxel_) {
        adaptiveVoxelGrid(cloud_filtered, frp_active);
    } else {
        pcl::VoxelGrid<PointType> vg;
        vg.setInputCloud(cloud_filtered);
        float leaf = frp_active ? static_cast<float>(frp_coarse_leaf_size_) : 0.05f;
        vg.setLeafSize(leaf, leaf, leaf);
        vg.filter(*cloud_filtered);
    }

    if (enable_sor_ && !frp_active) {
        applySOR(cloud_filtered);
    }
}
