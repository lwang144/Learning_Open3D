#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

/*
[Possion reconstruction] is a very intuitive method. Its core idea 
is that the point cloud represents the position of the surface of 
the object, and its normal vector represents the direction of the 
inside and outside. By implicitly fitting an indicator function 
derived from the object, a smooth object surface estimate can be given.
*/

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
    open3d::visualization::DrawGeometries({cloud_ptr}, "Original PointCloud");

    open3d::utility::LogInfo("Run Poisson surface reconstruction...");
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    std::vector<double> dentities;
    //----- Poisson Surface Reconstruction -----//
    std::tie(mesh, dentities) = open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*cloud_ptr,9);
    open3d::visualization::DrawGeometries({mesh}, "Possion Reconstruction");
    // [densities] return value that indicates for each vertex the density
    // Poisson surface reconstruction will also create triangles in 
    //areas of low point density, and even extrapolates into some areas

    //----- Density Visualization -----//
    open3d::utility::LogInfo("Visualize Densities");
    double maxDensity = *std::max_element(dentities.begin(), dentities.end());
    double minDensity = *std::min_element(dentities.begin(), dentities.end());
    std::cout <<" Max Density: " << maxDensity << ", Min Density: " << minDensity << std::endl;
    
    // Color mapping
    for (size_t i=0; i < dentities.size(); i++){
        double color = (dentities[i]-minDensity)/(maxDensity-minDensity);
        mesh -> vertex_colors_[i] = Eigen::Vector3d(color,0.5,0);
    }
    open3d::utility::LogInfo("Green indicates low density.");
    open3d::utility::LogInfo("Orange indicates high density.");
    open3d::visualization::DrawGeometries({mesh}, "Density Visualization");

    return 0;
}
