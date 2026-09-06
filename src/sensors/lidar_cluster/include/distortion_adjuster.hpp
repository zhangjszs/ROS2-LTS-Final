/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2020-02-25 14:38:12
 */

#pragma once

#include <common_msgs/msg/huat_ins_p2.hpp>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <deque>
#include <imu_data.hpp>
#include <mutex>
#include <thread>

#include "point_type.h"
namespace lidar_distortion {

class DistortionAdjuster {
   public:
    DistortionAdjuster();
    void SetMotionInfo(float scan_period, ImuData velocity_data);
    void AdjustCloud(pcl::PointCloud<PointType>::Ptr& input_cloud_ptr,
                     pcl::PointCloud<PointType>::Ptr& output_cloud_ptr);

   private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

   private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
    float head_;
    float pitch_;
};

}  // namespace lidar_distortion