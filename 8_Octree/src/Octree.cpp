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
    //auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    auto pcd = mesh -> SamplePointsPoissonDisk(2000);
    // Fit to unit cube
    Eigen::Vector3d ss = pcd->GetMaxBound() - pcd->GetMinBound();
    auto scale = ss.maxCoeff();
    std::cout << scale << std::endl;
    pcd ->Scale(1/scale, pcd->GetCenter());
    if (open3d::io::ReadTriangleMesh(argv[1], *mesh))
    {
        open3d::utility::LogInfo("Successfully read {}", argv[1]);
    }
    else
    {
        open3d::utility::LogInfo("Failed to read {}", argv[1]);
        return 1;
    }
    open3d::visualization::DrawGeometries({pcd});

    open3d::utility::LogInfo("Octree Division...");
    constexpr int max_depth = 4;
    auto octree = std::make_shared<open3d::geometry::Octree>(max_depth);
    octree-> ConvertFromPointCloud(*pcd);
    // open3d::visualization::DrawGeometries({octree});
    return 0;
} 