/*
 * @Description: 点云畸变补偿
 * @Author: Jiaxi Dai
 * @Date: 2021年11月29日14:52:37
 */
#include <distortion_adjuster.hpp>

#include <numbers>

using namespace lidar_distortion;
DistortionAdjuster::DistortionAdjuster() {}

void DistortionAdjuster::SetMotionInfo(float scan_period, ImuData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_acceleration.x, -velocity_data.linear_acceleration.y,
        -velocity_data.linear_acceleration.z;
    angular_rate_ << velocity_data.rpy.heading, -velocity_data.rpy.pitch, 0;
}

void DistortionAdjuster::AdjustCloud(pcl::PointCloud<PointType>::Ptr &input_cloud_ptr,
                                     pcl::PointCloud<PointType>::Ptr &output_cloud_ptr) {
    output_cloud_ptr.reset(new pcl::PointCloud<PointType>);
    if (!input_cloud_ptr || input_cloud_ptr->points.empty()) {
        return;
    }
    pcl::PointCloud<PointType>::Ptr origin_cloud_ptr = input_cloud_ptr;
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kTwoPi = 2.0f * kPi;
    float orientation_space = kTwoPi;
    float delete_space = 5.0f * kPi / 180.0f;
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3, 3>(0, 0) = rotate_matrix.inverse();
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);
    // 分别旋转线速度和角速度到扫描起始帧坐标系，保持两者独立
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0f)
            orientation += kTwoPi;

        if (orientation < delete_space || kTwoPi - orientation < delete_space)
            continue;
        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;
        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x, origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);
        // 旋转补偿 + 线速度补偿叠加，两者缺一不可
        Eigen::Vector3f adjusted_point = current_matrix * origin_point + velocity_ * real_time;
        PointType point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return;
}

Eigen::Matrix3f DistortionAdjuster::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(0), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy;
    return t_V.matrix();
}