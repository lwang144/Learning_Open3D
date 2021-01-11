#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

// The voxel grid can also be created from a point cloud.
int main(int argc, char* argv[]) 
{
    //----- Create a pointer to store the point cloud -----//
    auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
    open3d::visualization::Visualizer visualizer; //Create a visualizer object
    // Create a pointcloud from a file Return an empty pointcloud, if fail to read the file.
    cloud_ptr = open3d::io::CreatePointCloudFromFile(argv[1]);
    if (open3d::io::ReadPointCloud(argv[1], *cloud_ptr))
    {
        open3d::utility::LogInfo("Successfully read {}", argv[1]);
    }
    else
    {
        open3d::utility::LogInfo("Failed to read {}", argv[1]);
        return 1;
    }
    auto voxel = open3d::geometry::VoxelGrid::CreateFromPointCloud(*cloud_ptr, 0.01);
    open3d::visualization::DrawGeometries({cloud_ptr, voxel});

    auto mesh = open3d::io::CreateMeshFromFile(argv[2]);
    open3d::utility::LogInfo("Mesh has {:d} vertices, {:d} triangles.",
                        mesh -> vertices_.size(), mesh -> triangles_.size());
    mesh -> RemoveDuplicatedVertices();
    mesh -> RemoveDuplicatedTriangles();
    mesh -> RemoveDegenerateTriangles();
    mesh -> RemoveUnreferencedVertices();
    open3d::utility::LogInfo(
                "After purge vertices, Mesh1 has {:d} vertices, {:d} "
                "triangles.",
                mesh -> vertices_.size(), mesh -> triangles_.size());

    // Fit to the unit cube
    auto scale = mesh->GetMaxBound() - mesh->GetMinBound();
    //std::cout << "Here is mat.maxCoeff():  " << scale.maxCoeff() << std::endl;
    auto center = mesh->GetCenter();
    mesh -> Scale(1/scale.maxCoeff(), center);

    open3d::visualization::DrawGeometries({mesh});

    std::cout << "Voxelization: " << std::endl;
    auto voxel1 = open3d::geometry::VoxelGrid::CreateFromTriangleMesh(*mesh, 0.05);
    open3d::visualization::DrawGeometries({mesh, voxel1});
    
    return 0;
}