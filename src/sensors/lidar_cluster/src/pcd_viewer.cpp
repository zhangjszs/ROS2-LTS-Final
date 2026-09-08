
#include <lidar_cluster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <format>

using std::cout;
using std::endl;
using std::string;

void VoxelFilter(const pcl::PointCloud<PointType>::Ptr input, pcl::PointCloud<PointType>::Ptr& output) {
    auto startTimePassThrough = std::chrono::steady_clock::now();
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*output);
    auto endTimePassThrough = std::chrono::steady_clock::now();
    auto elapsedTimePassThrough =
        std::chrono::duration_cast<std::chrono::microseconds>(endTimePassThrough - startTimePassThrough);
    std::cout << "Voxel filter time: " << elapsedTimePassThrough.count() << " us" << std::endl;
    return;
}

int main() {
    //***************************读取 PCD 文件*****************************************
    // 在一个视图下同时显示两张点云图像
    // std::shared_ptr<lidar_cluster> lc_ = new()  // 已注释：不使用的代码

    pcl::PointCloud<PointType>::Ptr source(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr source2(new pcl::PointCloud<PointType>());

    pcl::PointCloud<PointType>::Ptr source3(new pcl::PointCloud<PointType>());
    // 输入点云路径
    string filename1 = "/home/adams/postsynced_msf/pcdtest/filter/ground_out.pcd";
    string filename2 = "/home/adams/postsynced_msf/pcdtest/filter/StatisticalOutlierFilter.pcd";

    pcl::io::loadPCDFile(filename1, *source);
    cout << "Point cloud loaded!" << endl;

    VoxelFilter(source, source2);
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->initCameraParameters();

    // 在一个视图里显示两张点云图
    int v1(0);
    viewer->createViewPort(0.0, 0.0, 1.0 / 2.0, 1.0, v1);
    // viewer->setBackgroundColor(28, 28, 28, v1);  // 已注释：设置背景色
    viewer->addText(std::format("before VoxelGridFiltering        number of points: {}", source->points.size()), 10, 10,
                    20, 150, 150, 150, "v1 text", v1);
    // viewer->addText(s.str(), 10, 10, "v1 text", v1);  // 已注释：添加文本
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb1(source, 255, 255, 255);
    viewer->addPointCloud<PointType>(source, rgb1, "sample cloud1", v1);

    int v2(0);
    viewer->createViewPort(1.0 / 2.0, 0.0, 1.0, 1.0, v2);
    // viewer->setBackgroundColor(28, 28, 28, v2);  // 已注释：设置背景色
    viewer->addText(std::format("after VoxelGridFiltering        number of points: {}", source2->points.size()), 10, 10,
                    20, 150, 150, 150, "v2 text", v2);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> rgb2(source2, 255, 255, 255);
    viewer->addPointCloud<PointType>(source2, rgb2, "sample cloud2", v2);

    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud1");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud2");

    viewer->addCoordinateSystem(1.0);
    viewer->spin();
    return 0;
}