#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>

#include <Eigen/Dense>

#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"

// Load 3 point clouds form folder 'test_data/ICP' 
auto load_point_cloud(const double &voxel_size = 0.0){
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> PCDs;
    std::cout << '\n' << "[ 1. Load Pointcloud: ]" << std::endl;
    for(uint8_t i = 0; i < 3; i ++){
        std::string filePath = "../../test_data/ICP/cloud_bin_" + std::to_string(i) + ".pcd";
        std::shared_ptr<open3d::geometry::PointCloud> pcd;
        pcd = open3d::io::CreatePointCloudFromFile(filePath);
        auto pcd_down = pcd -> VoxelDownSample(voxel_size);
        PCDs.push_back(pcd_down);
        open3d::utility::LogInfo("cloud_bin_{:d}, down sampling finished.", i);
    }
    return PCDs;
}

auto pairwise_registration( const open3d::geometry::PointCloud &source,
                            const open3d::geometry::PointCloud &target,
                            const double max_correspondence_distance_coarse,
                            const double max_correspondence_distance_fine){
    open3d::utility::LogInfo("Apply Point-to-plane ICP");
    //----- Coarse registration -----//
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(source, target, max_correspondence_distance_coarse,
                            Eigen::MatrixXd::Identity(4,4), open3d::pipelines::registration::TransformationEstimationPointToPlane());
    //----- Fine registration -----//
    auto icp_fine   = open3d::pipelines::registration::RegistrationICP(source, target, max_correspondence_distance_fine,
                            icp_coarse.transformation_, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    auto transformationICP = icp_fine.transformation_;
    auto informationICP = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target, 
                                        max_correspondence_distance_fine, icp_fine.transformation_);
    return std::make_tuple(transformationICP, informationICP);
}

auto full_registration( const std::vector<std::shared_ptr<open3d::geometry::PointCloud>> &PCDs,
                        const double max_correspondence_distance_coarse,
                        const double max_correspondence_distance_fine){
    auto pose_graph = open3d::pipelines::registration::PoseGraph();
    auto odomentry  = Eigen::MatrixXd::Identity(4,4);
    pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odomentry));
    for(int sourceID = 0; sourceID < PCDs.size(); sourceID ++){
        for(int targetID = sourceID + 1; targetID < PCDs.size(); targetID ++){
            Eigen::Matrix4d transformationICP; 
            Eigen::Matrix6d_u informationICP;
            std::tie(transformationICP, informationICP) = pairwise_registration(*PCDs[sourceID], *PCDs[targetID], 
                                                            max_correspondence_distance_coarse, max_correspondence_distance_fine);
            open3d::utility::LogInfo("   Build open3d::pipelines::registration::PoseGraph");
            if(targetID == sourceID + 1){
                open3d::utility::LogInfo("   PCD[{:d}, {:d}], --Odomentry Edge", sourceID, targetID);
                auto odomentry_temp = transformationICP.cwiseProduct(odomentry); // Dot product
                pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odomentry_temp.inverse()));
                pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(sourceID, targetID, 
                                                                    transformationICP, informationICP, false));
            }
            else{
                open3d::utility::LogInfo("   PCD[{:d}, {:d}], --Loop Closure Edge", sourceID, targetID);
                pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(sourceID, targetID, 
                                                                    transformationICP, informationICP, true));
            }
        }
    }
    return pose_graph;
}

int main(){
    //----- Prepare Point cloud -----//
    double voxel_size = 0.02;
    auto PCD_down = load_point_cloud(voxel_size);

    //----- Full Registration -----//
    std::cout << '\n' << "[ 2. Full Registration: ]" << std::endl;
    double max_correspondence_distance_coarse = voxel_size * 15;
    double max_correspondence_distance_fine = voxel_size * 1.5;
    auto pose_graph = full_registration(PCD_down, max_correspondence_distance_coarse, max_correspondence_distance_fine);

    //----- Optimizing PoseGraph -----//
    std::cout << '\n' << "[ 3. Optimizing PoseGraph: ]" << std::endl;
    auto option =open3d::pipelines::registration::GlobalOptimizationOption(max_correspondence_distance_fine, 0.25, 0);
    open3d::pipelines::registration::GlobalOptimization(pose_graph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
                                                        open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(), option);

    //----- Transform points and display -----//
    std::cout << '\n' << "[ 4. Transform points and display: ]" << std::endl;
    for(int pointID = 0; pointID < PCD_down.size(); pointID ++){
        open3d::utility::LogInfo("PCD_{:d}", pointID);

    }
    return 0;
}