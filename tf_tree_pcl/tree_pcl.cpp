#include <iostream>
#include <pcl/common//common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>


int main(int argc, char** argv, char** env) {
    PCL_INFO("%s v0.1.0", argv[0]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    const std::string ply_path = "../data/bunny.ply/bunny.ply";

    if (pcl::io::loadPLYFile(ply_path, *cloud_ptr) != 0) {
        PCL_ERROR("load ply failed, path: %s.\n", ply_path);
        return -1;
    }

    PCL_INFO("points num: %d.\n", cloud_ptr->points.size());

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_ptr);

    pcl::PointXYZ search_pt;
    search_pt.x = -0.0168404;
    search_pt.y = 0.11015400;
    search_pt.z = -0.00153695;

    auto k = 10;

    std::vector<int> k_indices(k);
    std::vector<float> k_sqrt_dist(k);

    if (kdtree.nearestKSearch(search_pt, k, k_indices, k_sqrt_dist) > 0) {
        for (int idx = 0; idx < k; ++idx) {
            PCL_INFO("k_point(%f,%f,%f) dist: %f.\n",
                     cloud_ptr->points[k_indices[idx]].x,
                     cloud_ptr->points[k_indices[idx]].y,
                     cloud_ptr->points[k_indices[idx]].z,
                     k_sqrt_dist[idx]);
        }
    }

    std::vector<int> k_radius_indices;
    std::vector<float> k_sqrt_radius_dist;

    auto radius = 0.2;

    if (kdtree.radiusSearch(search_pt, radius, k_radius_indices, k_sqrt_radius_dist) > 0) {
        for (int idx = 0; idx < k_radius_indices.size(); ++idx) {
            PCL_INFO("radius_point(%f,%f,%f) dist: %f.\n",
                     cloud_ptr->points[k_radius_indices[idx]].x,
                     cloud_ptr->points[k_radius_indices[idx]].y,
                     cloud_ptr->points[k_radius_indices[idx]].z,
                     k_sqrt_radius_dist[idx]);
        }
    }

    return 0;
}