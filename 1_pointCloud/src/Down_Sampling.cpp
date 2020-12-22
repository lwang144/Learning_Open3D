#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"


int main(int argc, char *argv[]) 
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

    //----- Voxel Down sampling -----//
    open3d::utility::LogInfo("Downsample the point cloud with a voxel of 0.05");
    auto downsampled = cloud_ptr -> VoxelDownSample(0.05);

    //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Open3D Down Sampling", 1600, 900);
    visualizer.AddGeometry(downsampled);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return 0;
}