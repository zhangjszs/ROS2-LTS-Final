#include <ground_segmentation/svd_ground_strategy.h>
#include <pcl/common/centroid.h>

#include <algorithm>
#include <rclcpp/rclcpp.hpp>

using namespace Eigen;

static void ExtractInitialSeeds(const pcl::PointCloud<PointType>& p_sorted, pcl::PointCloud<PointType>::Ptr seeds_pc,
                                int num_lpr, double th_seeds) {
    double sum = 0;
    int cnt = 0;
    for (int i = 0; i < static_cast<int>(p_sorted.points.size()) && cnt < num_lpr; i++) {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;
    seeds_pc->clear();
    seeds_pc->points.reserve(p_sorted.points.size());
    for (int i = 0; i < static_cast<int>(p_sorted.points.size()); i++) {
        if (p_sorted.points[i].z < lpr_height + th_seeds) {
            seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
}

static void EstimateGroundPlane(const pcl::PointCloud<PointType>::Ptr& ground_pc, VectorXf& normal, float& d,
                                float& th_dist_d, double th_dist) {
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    Eigen::Vector4f pc_mean = Eigen::Vector4f::Zero();
    pcl::computeMeanAndCovarianceMatrix(*ground_pc, cov, pc_mean);
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal = (svd.matrixU().col(2));
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d = -(normal.transpose() * seeds_mean)(0, 0);
    th_dist_d = static_cast<float>(th_dist) - d;
}

void SvdGroundStrategy::segment(const pcl::PointCloud<PointType>::Ptr& input, pcl::PointCloud<PointType>::Ptr& ground,
                                pcl::PointCloud<PointType>::Ptr& not_ground, const GroundSegParams& params) {
    not_ground->points.clear();
    pcl::PointCloud<PointType> laserCloudIn = *input;
    const pcl::PointCloud<PointType>& laserCloudIn_org = *input;

    // P2-A: 先用 remove_if 过滤地面以下极低点（O(N)），再用 nth_element 做部分排序（O(N)）
    // 替代原来的全量 std::sort（O(N logN)），只需保证前 num_lpr 个元素是最小的即可
    const float z_floor = -1.0f * static_cast<float>(params.sensor_height);
    std::erase_if(laserCloudIn.points, [z_floor](const PointType& p) { return p.z < z_floor; });

    if (!laserCloudIn.points.empty()) {
        const size_t nth = std::min(static_cast<size_t>(params.num_lpr), laserCloudIn.points.size() - 1);
        std::ranges::nth_element(laserCloudIn.points.begin(), laserCloudIn.points.begin() + nth,
                                 laserCloudIn.points.end(), {}, &PointType::z);
    }

    auto seeds_pc = std::make_shared<pcl::PointCloud<PointType>>();
    ExtractInitialSeeds(laserCloudIn, seeds_pc, params.num_lpr, params.th_seeds);

    ground = seeds_pc;

    MatrixXf points(laserCloudIn_org.points.size(), 3);
    {
        int j = 0;
        for (const auto& p : laserCloudIn_org.points) {
            points.row(j++) << p.x, p.y, p.z;
        }
    }

    VectorXf normal;
    float d = 0.0f;
    float th_dist_d = 0.0f;
    const int n_pts = static_cast<int>(laserCloudIn_org.points.size());
    VectorXf result(n_pts);

    for (int i = 0; i < params.num_iter; i++) {
        EstimateGroundPlane(ground, normal, d, th_dist_d, params.th_dist);
        const bool last = (i + 1 == params.num_iter);
        ground->points.clear();
        ground->points.reserve(static_cast<size_t>(n_pts));
        if (last) {
            not_ground->points.clear();
            not_ground->points.reserve(static_cast<size_t>(n_pts));
        }

        result.noalias() = points * normal;
        for (int r = 0; r < n_pts; r++) {
            if (result[r] < th_dist_d) {
                ground->points.push_back(laserCloudIn_org[r]);
            } else if (last) {
                not_ground->points.push_back(laserCloudIn_org[r]);
            }
        }
    }
}
