#pragma once

#include <pcl/point_cloud.h>
#include <rclcpp/rclcpp.hpp>

#include "point_type.h"

class PointCloudPreprocessor {
   public:
    explicit PointCloudPreprocessor(rclcpp::Node::SharedPtr node);

    void process(pcl::PointCloud<PointType>::Ptr& cloud, bool frp_active, double current_pitch, double current_speed);

    struct RoiBounds {
        double x_min = 0, x_max = 0, y_min = 0, y_max = 0, z_min = 0, z_max = 0;
    };

   private:
    void splitString(const std::string& in_string, std::vector<double>& out_array);
    void GetBaseXYBounds(double& x_min, double& x_max, double& y_min, double& y_max) const;
    RoiBounds computeDynamicROIBounds(double current_pitch, double current_speed) const;
    void adaptiveVoxelGrid(pcl::PointCloud<PointType>::Ptr& cloud, bool frp_active);
    void applySOR(pcl::PointCloud<PointType>::Ptr& cloud);

    void LoadZParams(rclcpp::Node::SharedPtr node);
    void LoadROIParams(rclcpp::Node::SharedPtr node);
    void LoadVoxelParams(rclcpp::Node::SharedPtr node);

    int road_type_ = 2;
    bool enable_dynamic_roi_ = false;
    bool pitch_compensation_ = true;
    bool speed_compensation_ = true;
    double z_pitch_scale_ = 0.1;
    double speed_x_scale_ = 0.5;
    double speed_y_shrink_ = 0.1;
    double min_roi_x_max_ = 20.0;
    double max_roi_x_max_ = 60.0;
    double z_down_ = -1.0;
    double z_up_ = 0.7;

    double accel_x_max_ = 0;
    double accel_x_min_ = 0;
    double accel_y_max_ = 0;
    double accel_y_min_ = 0;
    double track_x_max_ = 0;
    double track_x_min_ = 0;
    double track_y_max_ = 0;
    double track_y_min_ = 0;

    bool enable_adaptive_voxel_ = false;
    std::string voxel_ranges_str_;
    std::string voxel_leaf_sizes_str_;
    std::vector<double> voxel_ranges_;
    std::vector<double> voxel_leaf_sizes_;

    bool enable_sor_ = false;
    int sor_mean_k_ = 10;
    double sor_stddev_ = 1.0;

    double frp_coarse_leaf_size_ = 0.08;

    rclcpp::Clock clock_;
};
