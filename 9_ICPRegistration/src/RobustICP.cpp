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

auto apply_noise(const open3d::geometry::PointCloud &pcd,
                 const double &mu, const double &sigma){
    auto noisy_pcd = std::make_shared<open3d::geometry::PointCloud>();
    *noisy_pcd = pcd;
    std::vector<Eigen::Vector3d> points = noisy_pcd -> points_;
    // Add Gaussian noise:
    for(size_t i=0; i < points.size(); i++){
        Eigen::Vector3d random_noise;
        random_noise << (mu + (double)sigma*(-1 + 2 * (rand()/(double)(RAND_MAX)))), 
                        (mu + (double)sigma*(-1 + 2 * (rand()/(double)(RAND_MAX)))),
                        (mu + (double)sigma*(-1 + 2 * (rand()/(double)(RAND_MAX))));
        points[i]= points[i] + random_noise;
    }
    noisy_pcd -> points_ = points;
    return noisy_pcd;
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
    std::cout << "Trans_init:" << std::endl;
    //draw_registration_result(*source, *target, trans_init);

    std::cout << "--------------------------------------------------" << std::endl; 
    std::cout << "Source Pointcloud + noise:" << std::endl;
    double mu = 0;
    double sigma = 0.1;
    auto source_noisy = apply_noise(*source, mu, sigma);
    open3d::visualization::DrawGeometries({source_noisy}, "Noisy PointCloud");
    
    std::cout << "--------------------------------------------------" << std::endl;
    double ICP_threshold = 0.02;
    std::cout << "Vanilla point-to-plane ICP, threshold= " << ICP_threshold << std::endl;
    auto reg_p2p = open3d::pipelines::registration::RegistrationICP(*source, *target, ICP_threshold,
                trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint());
    auto evaluation = open3d::pipelines::registration::EvaluateRegistration(*source_noisy, *target, ICP_threshold, reg_p2p.transformation_);
    std::cout << "  fitness: " << evaluation.fitness_  << " (The higher, the better)"<< std::endl;
    std::cout << "  inlier_rms: " << evaluation.inlier_rmse_  << " (The lower, the better)"<< std::endl;
    draw_registration_result(*source, *target, reg_p2p.transformation_);

    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Robust point-to-plane ICP, threshold= " << ICP_threshold << std::endl;
    auto loss = open3d::pipelines::registration::TukeyLoss::TukeyLoss(0.1);  // ????
    // auto reg_p2l = open3d::pipelines::registration::RegistrationICP(*source, *target, ICP_threshold,
    //             trans_init, open3d::pipelines::registration::TransformationEstimationPointToPoint(
    //                 open3d::pipelines::registration::TukeyLoss::TukeyLoss(0.02)));

    return 0;
}