#include <iostream>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;


bool visualize_pcl(const PointCloudPtr& reference_pcl,
                   const PointCloudPtr& target_pcl,
                   const PointCloudPtr& registered_pcl) {
    pcl::visualization::PCLVisualizer viewer;
    pcl::visualization::PointCloudColorHandlerCustom<PointT> red(reference_pcl, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> green(target_pcl, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> blue(registered_pcl, 0, 0, 255);

    viewer.addPointCloud(reference_pcl, red, "reference pcl");
    viewer.addPointCloud(target_pcl, green, "target pcl");
    viewer.addPointCloud(registered_pcl, blue, "registered pcl");

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return true;
}


bool calc_fpfh(const PointCloudPtr& src_pcl_ptr,
               PointCloudPtr& dst_pcl_ptr,
               pcl::PointCloud<pcl::PointNormal>::Ptr& normal_ptr,
               pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfh_descriptor) {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*src_pcl_ptr, *dst_pcl_ptr, indices);
    std::cout << "src pcl points: " << src_pcl_ptr->points.size() << std::endl;
    std::cout << "after nan, pcl points: " << dst_pcl_ptr->points.size() << std::endl;

    // 下采样
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud(dst_pcl_ptr);
    const float leaf_size = 0.012f;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*dst_pcl_ptr);
    PCL_INFO("after voxel grid filter, points: %d.\n", dst_pcl_ptr->points.size());

    // 计算表面法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr norm_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    norm_est.setInputCloud(dst_pcl_ptr);
    norm_est.setSearchMethod(norm_kdtree);
    norm_est.setRadiusSearch(0.02);
    norm_est.compute(*normal_ptr);
    PCL_INFO("norm points: %d.\n", normal_ptr->points.size());

    // 计算FPFH
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr fpfh_kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    fpfh_est.setInputCloud(dst_pcl_ptr);
    fpfh_est.setInputNormals(normal_ptr);
    fpfh_est.setSearchMethod(fpfh_kdtree);
    fpfh_est.setRadiusSearch(0.05);
    fpfh_est.compute(*fpfh_descriptor);
    PCL_INFO("fpfh points: %d.\n", fpfh_descriptor->points.size());
    return true;
}


int main(int argc, char** argv, char** env) {
    PCL_INFO("%s v0.1.0", argv[0]);

    const std::string reference_pcd_path = "../data/icp_bunny/source.pcd";
    const std::string target_pcd_path = "../data/icp_bunny/target.pcd";
    const std::string scia_reg_pcd_path = "../data/icp_bunny/scia_registration.pcd";
    const std::string icp_reg_pcd_path = "../data/icp_bunny/icp_registration.pcd";

    PointCloudPtr reference_pcl_ptr(new PointCloud);
    PointCloudPtr target_pcl_ptr(new PointCloud);
    PointCloudPtr registered_pcl_ptr(new PointCloud);

    if (pcl::io::loadPCDFile<PointT>(reference_pcd_path, *reference_pcl_ptr) == -1) {
        PCL_ERROR("Couldn't read file reference_pcd_path: \s.\n", reference_pcd_path);
        return -1;
    }

    if (pcl::io::loadPCDFile<PointT>(target_pcd_path, *target_pcl_ptr) == -1) {
        PCL_ERROR("Couldn't read file target_pcd_path: \s.\n", target_pcd_path);
        return -1;
    }

    pcl::console::TicToc tt;

    pcl::PointCloud<pcl::PointNormal>::Ptr ref_norm_ptr(new pcl::PointCloud<pcl::PointNormal>);
    PointCloudPtr ref_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr ref_fpfh_ptr(new pcl::PointCloud<pcl::FPFHSignature33>);

    pcl::PointCloud<pcl::PointNormal>::Ptr target_norm_ptr(new pcl::PointCloud<pcl::PointNormal>);
    PointCloudPtr target_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_fpfh_ptr(new
            pcl::PointCloud<pcl::FPFHSignature33>);

    calc_fpfh(reference_pcl_ptr, ref_ptr, ref_norm_ptr, ref_fpfh_ptr);
    calc_fpfh(target_pcl_ptr, target_ptr, target_norm_ptr, target_fpfh_ptr);


    // SAmple Consensu 粗匹配
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
    scia.setInputSource(ref_ptr);
    scia.setInputTarget(target_ptr);
    scia.setSourceFeatures(ref_fpfh_ptr);
    scia.setTargetFeatures(target_fpfh_ptr);
    //scia.setMinSampleDistance(1);
    //scia.setNumberOfSamples(2);
    //scia.setCorrespondenceRandomness(20);

    PointCloudPtr scia_reg_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    scia.align(*scia_reg_ptr);

    std::cout << "sac has converged: " << scia.hasConverged() << " score: "
              << scia.getFitnessScore() << std::endl;

    Eigen::Matrix4f sac_trans = scia.getFinalTransformation();
    std::cout << "scac transformation: " << std::endl << sac_trans << std::endl;
    pcl::io::savePCDFileASCII(scia_reg_pcd_path, *scia_reg_ptr);

    // icp 精匹配
    PointCloudPtr icp_reg_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(ref_ptr);
    icp.setInputTarget(target_ptr);
    icp.setMaxCorrespondenceDistance(0.04);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(0.2);
    icp.align(*icp_reg_ptr, sac_trans);
    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    std::cout << "icp transformation: " << std::endl << icp_trans << std::endl;
    pcl::io::savePCDFileASCII(icp_reg_pcd_path, *icp_reg_ptr);

    // 可视化
    visualize_pcl(ref_ptr, target_ptr, icp_reg_ptr);

    std::cout << "[time consume: " << tt.toc() / 1000.0 << " s.]" << std::endl;

    return 0;
}