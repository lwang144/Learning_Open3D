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

    //----- Statistical outlier removal -----//
    //std::shared_ptr<open3d::geometry::PointCloud> cl;
    //std::vector<size_t> ind;
    //std::tie(cl, ind) = cloud_ptr -> RemoveStatisticalOutliers(20, 2.0); // nb_neighbors, std_radio

    //----- Radius outlier removal -----//
    std::shared_ptr<open3d::geometry::PointCloud> cl;
    std::vector<size_t> ind;
    std::tie(cl, ind) = cloud_ptr -> RemoveRadiusOutliers(16, 0.05); // nb_points, std_radius
    
    std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = cloud_ptr -> SelectByIndex(ind);
    std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = cloud_ptr -> SelectByIndex(ind, true);
    inlier_cloud -> PaintUniformColor(Eigen::Vector3d(0.8, 0.8, 0.8));
    outlier_cloud -> PaintUniformColor(Eigen::Vector3d(1, 0, 0));    

    //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Open3D Down Sampling", 1600, 900);
    visualizer.AddGeometry(inlier_cloud);
    visualizer.AddGeometry(outlier_cloud);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return 0;
}