#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>

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
    open3d::visualization::DrawGeometries({source_temp, target_temp}, "Registration result");
}

auto preprocess_point_cloud(const open3d::geometry::PointCloud &pcd,
                            const double &voxel_size){
    open3d::utility::LogInfo("Preprocess :: Downsampling");
    std::shared_ptr<open3d::geometry::PointCloud> pcd_temp(new open3d::geometry::PointCloud);
    *pcd_temp = pcd;
    auto pcd_down = pcd_temp -> VoxelDownSample(voxel_size);
    open3d::utility::LogInfo("Preprocess :: Estimate normal");
    double radius_normal = voxel_size * 2;
    pcd_down -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, 30));
    double radius_feature = voxel_size * 5;
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down,
                        open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
    return std::make_tuple(pcd_down, pcd_fpfh);
}

auto global_registration(const open3d::geometry::PointCloud &source_down,
                                 const open3d::geometry::PointCloud &target_down,
                                 const open3d::pipelines::registration::Feature &source_fpfh,
                                 const open3d::pipelines::registration::Feature &target_fpfh, double voxel_size){
    double distance_threshold = voxel_size * 1.5;
    open3d::utility::LogInfo("Global Registration :: RANSAC registration on downsampled point clouds");
    // Prepare RANSAC checker
    std::vector<std::reference_wrapper<const open3d::pipelines::registration::CorrespondenceChecker>> correspondence_checker;
    auto correspondence_checker_edge_length = open3d::pipelines::registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
    auto correspondence_checker_distance = open3d::pipelines::registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
    correspondence_checker.push_back(correspondence_checker_edge_length);
    correspondence_checker.push_back(correspondence_checker_distance);
    auto RANSAC_result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source_down, 
                         target_down, source_fpfh, target_fpfh, true, distance_threshold, 
                         open3d::pipelines::registration::TransformationEstimationPointToPoint(false), 3,
                         correspondence_checker, open3d::pipelines::registration::RANSACConvergenceCriteria(100000, 0.999));
    return RANSAC_result;
}

auto refine_registration(const open3d::geometry::PointCloud &source,
                         const open3d::geometry::PointCloud &target,
                         const open3d::pipelines::registration::Feature &source_fpfh,
                         const open3d::pipelines::registration::Feature &target_fpfh, double voxel_size,
                         const open3d::pipelines::registration::RegistrationResult &RANSAC_result){
    double distance_threshold = voxel_size * 0.4;
    open3d::utility::LogInfo("Local Refinement :: Point-to-plane ICP registration is applied on original point");
    auto refine_result = open3d::pipelines::registration::RegistrationICP(source, target, distance_threshold, RANSAC_result.transformation_,
                                                                    open3d::pipelines::registration::TransformationEstimationPointToPlane());
    return refine_result;
}

auto fast_global_registration(const open3d::geometry::PointCloud &source_down,
                         const open3d::geometry::PointCloud &target_down,
                         const open3d::pipelines::registration::Feature &source_fpfh,
                         const open3d::pipelines::registration::Feature &target_fpfh, double voxel_size){
    double distance_threshold = voxel_size * 0.5;
    open3d::utility::LogInfo("Global Registration :: Apply fast global registration with distance threshold");
    auto result = open3d::pipelines::registration::RegistrationRANSACBasedOnFeatureMatching(source_down,
                    target_down, source_fpfh, target_fpfh, 
                    open3d::pipelines::registration::FastGlobalRegistrationOption::FastGlobalRegistrationOption(NULL, NULL, NULL, distance_threshold));
    return result;
}

double timer_cal(const std::chrono::_V2::system_clock::time_point &start_time){
    auto end_time = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    double time_passed = (double) duration.count() * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den;
    return time_passed;
}

int main(int argc, char* argv[]) 
{
    auto source = std::make_shared<open3d::geometry::PointCloud>();
    auto target = std::make_shared<open3d::geometry::PointCloud>();
    source = open3d::io::CreatePointCloudFromFile("../../test_data/ICP/cloud_bin_0.pcd");
    target = open3d::io::CreatePointCloudFromFile("../../test_data/ICP/cloud_bin_1.pcd");
    open3d::utility::LogInfo("PrepareData :: Load two point clouds and disturb initial pose...");
    Eigen::Matrix4d trans_init;
    trans_init << 0.0, 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0;
    std::cout << "Trans_init:" << '\n' << trans_init << std::endl;
    draw_registration_result(*source, *target, trans_init);

    double voxel_size = 0.05; // Mean 5 cm for this dataset
    std::shared_ptr<open3d::geometry::PointCloud> source_down, target_down;
    std::shared_ptr<open3d::pipelines::registration::Feature> source_fpfh, target_fpfh;
    std::tie(source_down, source_fpfh) = preprocess_point_cloud(*source, voxel_size);
    std::tie(target_down, target_fpfh) = preprocess_point_cloud(*target, voxel_size);

    //----- RANSAC -----//
    auto start_RANSAC = std::chrono::system_clock::now();  
    auto RANSAC_result = global_registration(*source_down, *target_down, *source_fpfh, *target_fpfh, voxel_size);
    double time_RANSAC = timer_cal(start_RANSAC);   // Calculate RANSAC time
    std::cout << "              --RANSAC time: " << time_RANSAC << " sec."<< std::endl;
    draw_registration_result(*source, *target, RANSAC_result.transformation_);

    //----- Local Refinement -----//
    auto refine_result = refine_registration(*source, *target, *source_fpfh, *target_fpfh, voxel_size, RANSAC_result);
    draw_registration_result(*source, *target, refine_result.transformation_);
    return 0;
}