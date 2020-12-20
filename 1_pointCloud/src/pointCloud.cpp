#include <iostream>
#include <memory>
#include <thread>
#include <Eigen/Dense>

#include "open3d/Open3D.h"

int main(int argc, char *argv[]) 
{
    // Create a pointer to store the point cloud
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

    // Returns 'true' if the point cloud contains points.
    bool pointcloud_has_normal = cloud_ptr->HasNormals();
    if(pointcloud_has_normal){ 
        open3d::utility::LogInfo("Pointcloud has normals.");
    }else{
        open3d::utility::LogInfo("Pointcloud has no normals.");
    }

    // Down sampling
    auto downsampled = cloud_ptr -> VoxelDownSample(0.05);

    // Get axis aligned/oriented bounding box
    open3d::geometry::AxisAlignedBoundingBox bounding_box_aligned = cloud_ptr -> GetAxisAlignedBoundingBox();
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> 

    // Point cloud Visualization
    //open3d::visualization::DrawGeometries({downsampled}, "PointCloud", 1600, 900);
    visualizer.CreateVisualizerWindow("Open3D Test", 1600, 900);
    visualizer.AddGeometry(downsampled);
    visualizer.AddGeometry(bounding_box_aligned);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return 0;
}