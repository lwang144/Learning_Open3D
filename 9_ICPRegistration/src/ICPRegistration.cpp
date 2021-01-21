#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

void draw_registration_result(const open3d::geometry::PointCloud &source,
                              const open3d::geometry::PointCloud &target,
                              const Eigen::Matrix4d &Transforamtion){
    std::shared_ptr<open3d::geometry::PointCloud> source_temp(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_temp(new open3d::geometry::PointCloud);
    *source_temp = source;
    *target_temp = target;
    source_temp -> PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
    target_temp -> PaintUniformColor(Eigen::Vector3d(0, 0.651, 0.929));
    source_temp -> Transform(Transforamtion);
    open3d::visualization::DrawGeometries({source_temp, target_temp},
                                  "Registration result");
}

int main(int argc, char* argv[]) 
{
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    source = open3d::io::CreatePointCloudFromFile("../../test_data/ICP/cloud_bin_0.pcd");
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    target = open3d::io::CreatePointCloudFromFile("../../test_data/ICP/cloud_bin_1.pcd");
    double threshold = 0.02;
    
    // Manually set the transformation matrix
    Eigen::Matrix4d trans_init;
    trans_init << 0.862,  0.011, -0.507,  0.5,
                 -0.139,  0.967, -0.215,  0.7,
                  0.487,  0.255,  0.835, -1.4,
                  0.0,    0.0,    0.0,    1.0;
    // std::cout << "Trans_init:" << '\n' << trans_init << std::endl;
    std::cout << "Initial alignment: " << std::endl;
    draw_registration_result(*source, *target, trans_init);
    auto evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, threshold, trans_init);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Point-to-point ICP: " << std::endl;
    auto reg_p2p = open3d::pipelines::registration::RegistrationICP(*source, *target, threshold,
                trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint());
    evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, threshold, reg_p2p.transformation_);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
    draw_registration_result(*source, *target, reg_p2p.transformation_);


    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "P-t-P ICP with 2000 iterations:" << std::endl;
    reg_p2p = open3d::pipelines::registration::RegistrationICP(*source, *target, threshold,
                trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint(),
                open3d::pipelines::registration::ICPConvergenceCriteria(0, 0, 2000));
    evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, threshold, reg_p2p.transformation_);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
    draw_registration_result(*source, *target, reg_p2p.transformation_);


    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "point-to-plane ICP:" << std::endl;
    reg_p2p = open3d::pipelines::registration::RegistrationICP(*source, *target, threshold,
                trans_init, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    evaluation = open3d::pipelines::registration::EvaluateRegistration(*source, *target, threshold, reg_p2p.transformation_);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
    draw_registration_result(*source, *target, reg_p2p.transformation_);
    return 0; 
}