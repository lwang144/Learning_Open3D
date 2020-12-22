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

    //----- DBSCAN Clustering -----//
    // [Group local point cloud clusters together]
    std::vector<int>labels = cloud_ptr -> ClusterDBSCAN(0.02, 10, true);

    int max_label = *max_element(labels.begin(),labels.end());
    open3d::utility::LogInfo("point cloud has {:d} clusters.", max_label+1);
    std::vector<Eigen::Vector3d> vColor; // Define color vector
	vColor.reserve(1 + max_label);
    // Generate random color
	for (int i = 0; i <= max_label; i++){
		double R = (rand()%(200-0)+0)/255.0;
		double G = (rand()%(128-100)+100)/255.0;
		double B = (rand()%(250-25)+25)/255.0;
		Eigen::Vector3d c = {R, G, B};
		vColor.push_back(c);
    }
    // Color each point cloud cluster
	std::vector<Eigen::Vector3d> pcColor;
	pcColor.reserve(labels.size());
    for (int i = 0; i < labels.size(); i++)
	{
		int label = labels[i];
		if (label <0){
			Eigen::Vector3d c = {0,0,0}; // noise=-1 set color to black
			pcColor.push_back(c);
		}
		else{
			Eigen::Vector3d c = vColor[label];
			pcColor.push_back(c);
		}
	}
    // Paint point cloud
    cloud_ptr -> colors_ = pcColor;

    //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Open3D Test", 1600, 900);
    visualizer.AddGeometry(cloud_ptr);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return 0;
}