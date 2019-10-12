#include <iostream>
#include <pcl/common//common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ PointItem;
typedef pcl::PointCloud<PointItem> PointCloud;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef PointCloud::Ptr PointCloudPtr;


int main(int argc, char** argv, char** env) {
    PCL_INFO("%s v0.1.0", argv[0]);

    const std::string pcd_path = "../data/rabbit.pcd";
    PointCloudPtr src_point_cloud_ptr(new PointCloud);

    if (pcl::io::loadPCDFile(pcd_path, *src_point_cloud_ptr) == -1) {
        PCL_ERROR("can not load pcd file, path: %s.\n", pcd_path);
        return -1;
    }

    PCL_INFO("calculate normals...");
    pcl::NormalEstimation<PointItem, pcl::PointNormal> nest;
    nest.setKSearch(50);
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    nest.setInputCloud(src_point_cloud_ptr);
    nest.compute(*normals);

    // µ„‘∆¡¨Ω”
    for (int i = 0; i < normals->points.size(); ++i) {
        normals->points[i].x = src_point_cloud_ptr->points[i].x;
        normals->points[i].y = src_point_cloud_ptr->points[i].y;
        normals->points[i].z = src_point_cloud_ptr->points[i].z;
    }

    pcl::visualization::PCLVisualizer viewer;
    // viewer.addPointCloud(src_point_cloud_ptr, "cloud");
    pcl::visualization::PointCloudColorHandlerCustom<PointItem> red(src_point_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud(src_point_cloud_ptr, red, "src_cloud");
    const int level = 10;
    float scale = 0.1;
    viewer.addPointCloudNormals<PointItem, pcl::PointNormal>(src_point_cloud_ptr, normals, level, scale,
            "normal");
    viewer.addCoordinateSystem();
    viewer.spin();
    return 0;
}