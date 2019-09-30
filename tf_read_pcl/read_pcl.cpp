#include <iostream>
#include <pcl/common//common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int read_pcd() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../data/test_pcd.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return -1;
    }

    std::cout << "Loaded "
              << cloud->width* cloud->height
              << " data points from test_pcd.pcd with the following fields: "
              << std::endl;

    for (size_t i = 0; i < cloud->points.size(); ++i)
        std::cout << "    " << cloud->points[i].x
                  << " " << cloud->points[i].y
                  << " " << cloud->points[i].z << std::endl;

    return 0;
}

int read_ply() {
    pcl::PLYReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    const std::string data_path = "../data/bunny.ply/bunny.ply";

    if (pcl::io::loadPLYFile(data_path, *cloud_ptr) != 0) {
        PCL_ERROR("load data failed, path: %s\n", data_path);
        return -1;
    }

    PCL_INFO("load %d points from %s.\n", cloud_ptr->points.size(), data_path);

    pcl::visualization::CloudViewer viewer("buuny viewer");
    viewer.showCloud(cloud_ptr);

    while (!viewer.wasStopped()) {

    }

    return 1;
}

int main(int argc, char** argv, char** env) {
    return read_ply();
}