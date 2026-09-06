#ifndef GROUND_SEGMENTATION_STRATEGY_H
#define GROUND_SEGMENTATION_STRATEGY_H

#include <pcl/point_cloud.h>

#include "point_type.h"

struct GroundSegParams {
    int num_iter = 3;
    int num_lpr = 5;
    double th_seeds = 0.03;
    double th_dist = 0.03;
    double sensor_height = 0.135;
    int max_iterations = 100;
    double distance_threshold = 0.03;
};

class GroundSegmentationStrategy {
   public:
    virtual ~GroundSegmentationStrategy() = default;
    virtual void segment(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& ground,
                         pcl::PointCloud<PointType>::Ptr& not_ground, const GroundSegParams& params) = 0;
};

#endif
