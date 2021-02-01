#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/ColoredICP.h"

void draw_registration_result(const open3d::geometry::PointCloud &source,
                              const open3d::geometry::PointCloud &target,
                              const Eigen::Matrix4d &Transforamtion){
    std::shared_ptr<open3d::geometry::PointCloud> source_temp(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_temp(new open3d::geometry::PointCloud);
    *source_temp = source;
    *target_temp = target;
    source_temp -> Transform(Transforamtion);
    open3d::visualization::DrawGeometries({source_temp, target_temp},
                                  "Registration result");
}

int main(int argc, char* argv[]) 
{
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    source = open3d::io::CreatePointCloudFromFile("../../test_data/ColoredICP/frag_115.ply");
    target = open3d::io::CreatePointCloudFromFile("../../test_data/ColoredICP/frag_116.ply");

    std::cout << "Load two point clouds and show initial pose." << std::endl;
    //Set the initial transformation matrix
    auto trans_init = Eigen::MatrixXd::Identity(4,4);
    draw_registration_result(*source, *target, trans_init);

    std::cout << "--------------------------------------------------" << std::endl; 
    std::cout << "Point-to-plane ICP registration, threshold=0.02" << std::endl;
    auto result_ICP = open3d::pipelines::registration::RegistrationICP(*source, *target, 0.02,
                trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint());
    auto evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, 0.02, result_ICP.transformation_);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
    std::cout << "[Geometric constraint does not prevent two planar surfaces from slipping.]" << std::endl;
    draw_registration_result(*source, *target, result_ICP.transformation_);
    
    std::cout << "--------------------------------------------------" << std::endl; 
    std::cout << "[3] Colored pointcloud registration" << std::endl;
    double voxel_radius[3] = {0.04, 0.02, 0.01};
    int max_iter[3] = {50, 30, 14};
    Eigen::Matrix4d_u current_transformation = Eigen::MatrixXd::Identity(4,4);
    open3d::pipelines::registration::RegistrationResult Result_ICP;
    for(int i=0; i < 3; i++){
        int iter = max_iter[i];
        double radius = voxel_radius[i];
        std::cout << "[" << iter << ", " << radius << ", " << i << "]" << std::endl;

        std::cout << "[3-1]. Downsample with a voxel size: " << radius << std::endl;
        auto source_down = source -> VoxelDownSample(radius);
        auto target_down = target -> VoxelDownSample(radius);

        std::cout << "[3-2]. Estimate normal. " << std::endl;
        source_down -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius*2.0, 30));
        target_down -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius*2.0, 30));

        std::cout << "[3-3]. Applying colored point cloud registration" << std::endl;
        Result_ICP = open3d::pipelines::registration::RegistrationColoredICP(
                            *source_down, *target_down, radius, current_transformation, 
                            open3d::pipelines::registration::TransformationEstimationForColoredICP(),
                            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, iter));
        current_transformation = Result_ICP.transformation_;
        evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, 0.02, Result_ICP.transformation_);
        std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
        std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
        std::cout << "------" << std::endl;
    }
    draw_registration_result(*source, *target, Result_ICP.transformation_);
    return 0;
}