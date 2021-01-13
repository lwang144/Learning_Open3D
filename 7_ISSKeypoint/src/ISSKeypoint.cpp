#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

int main(int argc, char* argv[]) 
{
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    mesh = open3d::io::CreateMeshFromFile(argv[1]);  // Load Triangle Mesh
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd -> points_ = mesh -> vertices_;

    // Compute the ISS Keypoints
    auto ISS_keypoints = std::make_shared<open3d::geometry::PointCloud>();
    open3d::utility::ScopeTimer timer("ISS keypoints Estimation");
    ISS_keypoints = open3d::geometry::keypoint::ComputeISSKeypoints(*pcd);
    open3d::utility::LogInfo("Detected {} keypoints", ISS_keypoints -> points_.size());

    // Visualization
    ISS_keypoints->PaintUniformColor(Eigen::Vector3d(1.0, 0.75, 0.0)); // Paint keypoints yellow
    mesh -> PaintUniformColor(Eigen::Vector3d(0.5, 0.5, 0.5));
    mesh -> ComputeVertexNormals();
    mesh -> ComputeTriangleNormals();
    open3d::visualization::DrawGeometries({mesh, ISS_keypoints}, "ISS Keypoints");
    return 0;
}