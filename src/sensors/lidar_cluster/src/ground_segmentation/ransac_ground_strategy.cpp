#include <ground_segmentation/ransac_ground_strategy.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <rclcpp/rclcpp.hpp>

void RansacGroundStrategy::segment(const pcl::PointCloud<PointType>::Ptr& input,
                                   pcl::PointCloud<PointType>::Ptr& ground, pcl::PointCloud<PointType>::Ptr& not_ground,
                                   const GroundSegParams& params) {
    pcl::SACSegmentation<PointType> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(params.max_iterations);
    seg.setDistanceThreshold(static_cast<float>(params.distance_threshold));

    seg.setInputCloud(input);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*ground);

    RCLCPP_DEBUG(rclcpp::get_logger("lidar_cluster"), "[lidar_cluster] Ground points: %zu", ground->points.size());

    extract.setNegative(true);
    extract.filter(*not_ground);
}
