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
    
    // Manually set the transformation matrix
    Eigen::Matrix4d_u 


    //
    std::cout << "Initial alignment: " << std::endl;
    auto evaluation = open3d::pipelines::registration::EvaluateRegistration();

    return 0; 
}