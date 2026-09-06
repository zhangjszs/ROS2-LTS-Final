#ifndef SVD_GROUND_STRATEGY_H
#define SVD_GROUND_STRATEGY_H

#include <ground_segmentation/ground_segmentation_strategy.h>

class SvdGroundStrategy : public GroundSegmentationStrategy {
   public:
    void segment(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& ground,
                 pcl::PointCloud<PointType>::Ptr& not_ground, const GroundSegParams& params) override;
};

#endif
