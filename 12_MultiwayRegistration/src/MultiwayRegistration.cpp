#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

// Load 3 point clouds form folder 'test_data/ICP'
auto load_point_cloud(const double &voxel_size = 0.0){
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> PCDs;
    for(uint8_t i = 0; i < 3; i ++){
        std::string filePath = "../../test_data/ICP/cloud_bin_" + std::to_string(i) + ".pcd";
        std::shared_ptr<open3d::geometry::PointCloud> pcd;
        pcd = open3d::io::CreatePointCloudFromFile(filePath);
        auto pcd_down = pcd -> VoxelDownSample(voxel_size);
        PCDs.push_back(pcd_down);
        open3d::utility::LogInfo("Load point cloud {:d}, down sample", i);
    }
    return PCDs;
}

auto pairwise_registration(const open3d::geometry::PointCloud &source,
                           const open3d::geometry::PointCloud &target,
                           const double max_correspondence_distance_coarse){
    open3d::utility::LogInfo("Apply Point-to-plane ICP");
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(source, target, max_correspondence_distance_coarse,
                            Eigen::MatrixXd::Identity(4,4),open3d::pipelines::registration::TransformationEstimationPointToPlane());
    auto icp_fine   = open3d::pipelines::registration::RegistrationICP();

}

int main(int argc, char* argv[]) 
{
    double voxel_size = 0.02;
    auto PCD = load_point_cloud(voxel_size);
    //open3d::visualization::DrawGeometries({PCD[2]}, "Registration result");
    std::cout << Eigen::MatrixXf::Identity(4,4) << std::endl;
    
    return 0;
}