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

    //----- Plane segmentation -----//
    double distance_threshold = 0.01;
    int ransac_n = 3;
    int num_iterations = 1000;    
    std::tuple<Eigen::Vector4d, std::vector<size_t>> vRes = 
                    cloud_ptr -> SegmentPlane(distance_threshold, ransac_n, num_iterations); // Return plane model and inliers
    // [a b c d] plane model
	Eigen::Vector4d para = std::get<0>(vRes);
    // Inliers
    std::vector<size_t> selectedIndex = std::get<1>(vRes);
    
    // Paint inliers red
    std::shared_ptr<open3d::geometry::PointCloud> inPC = cloud_ptr -> SelectByIndex(selectedIndex, false);
	const Eigen::Vector3d colorIn = {1,0,0};
	inPC->PaintUniformColor(colorIn);
    // Paint outliers black
	std::shared_ptr<open3d::geometry::PointCloud> outPC = cloud_ptr -> SelectByIndex(selectedIndex, true);
	const Eigen::Vector3d colorOut = { 0,0,0 };
	outPC->PaintUniformColor(colorOut);

    //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Open3D Test", 1600, 900);
    visualizer.AddGeometry(inPC);
    visualizer.AddGeometry(outPC);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return 0;
}