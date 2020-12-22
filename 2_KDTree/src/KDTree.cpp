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

    //----- Paint all points gray -----//
    open3d::utility::LogInfo("Paint all points Gray...");
    Eigen::Vector3d gray = {0.5, 0.5, 0.5}; 
    for(size_t i = 0; i < cloud_ptr->points_.size(); i++){
        cloud_ptr -> colors_[i] = gray;
    }
    //----- Build a KDTree -----//
    open3d::utility::LogInfo("Build KDTree...");
    open3d::geometry::KDTreeFlann kdtree;
    kdtree.SetGeometry(*cloud_ptr);

    // Pick the 1500th points as the anchor point and paint it red
    open3d::utility::LogInfo("Paint the 1500th point red.");
    cloud_ptr -> colors_[1500] = Eigen::Vector3d(1, 0, 0);


    //----- K Nearest Neighbors Search -----//
    open3d::utility::LogInfo("Find its 200 nearest neighbors, and paint them blue.");
    int knn = 200;
    std::vector<int> new_indices_vec(knn);
    std::vector<double> new_dists_vec(knn);
    kdtree.SearchKNN(cloud_ptr->points_[1500], knn, new_indices_vec, new_dists_vec);
    for (size_t i = 1; i < new_indices_vec.size(); i++) {
        //open3d::utility::LogInfo("{:d}, {:f}", (int)new_indices_vec[i], sqrt(new_dists_vec[i]));
        cloud_ptr -> colors_[new_indices_vec[i]] = Eigen::Vector3d(0, 0, 1);
    } // Skip the first index since it is the anchor point itself.


    //----- Radius Nearest Neighbors Search -----//
    //query all points with distances to the anchor point less than a given radius. 
    //paint these points with a green color.
    double radius = 0.2;
    std::vector<int> indices_vec(knn);
    std::vector<double> dists_vec(knn);
    kdtree.SearchRadius(cloud_ptr->points_[1500], radius, indices_vec, dists_vec);
    for (size_t i = 1; i < indices_vec.size(); i++) {
        cloud_ptr -> colors_[indices_vec[i]] = Eigen::Vector3d(0, 1, 0);
    } // Skip the first index since it is the anchor point itself.  


    //----- Hybrid search -----//
    // Radius K Neasrest Neighbors Search (Hybrid search)
    std::vector<int> hybrid_indices_vec(knn);
    std::vector<double> hybrid_dists_vec(knn);
    kdtree.SearchHybrid(cloud_ptr->points_[1500], radius, 50, hybrid_indices_vec, hybrid_dists_vec);
    for (size_t i = 1; i < hybrid_indices_vec.size(); i++) {
        cloud_ptr -> colors_[hybrid_indices_vec[i]] = Eigen::Vector3d(0, 1, 0.8);
    } // Skip the first index since it is the anchor point itself.  


    //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Open3D Down Sampling", 1600, 900);
    visualizer.AddGeometry(cloud_ptr);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return 0;
}