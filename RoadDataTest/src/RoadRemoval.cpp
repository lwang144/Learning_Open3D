#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"
#include "open3d/geometry/BoundingVolume.h"

auto plane_segmentation(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                        const double &distance_threshold = 0.2, 
                        const int &ransac_n = 3,
                        const int &num_iterations = 100){
    std::tuple<Eigen::Vector4d, std::vector<size_t>> vRes = 
                    pcd -> SegmentPlane(distance_threshold, ransac_n, num_iterations); // Return plane model and inliers
    // [a b c d] plane model
	Eigen::Vector4d para = std::get<0>(vRes);
    // Inliers
    std::vector<size_t> selectedIndex = std::get<1>(vRes);
    return selectedIndex;
}

auto StaticalOutlierRemoval(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                            const size_t &nb_neighbor = 20, 
                            const double &std_ratio = 2.0){
     //----- Statistical outlier removal -----//
    std::shared_ptr<open3d::geometry::PointCloud> cl;
    std::vector<size_t> ind;
    // nb_neighbors: Specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
    // std_ratio: The lower this number the more aggressive the filter will be.
    std::tie(cl, ind) = pcd -> RemoveStatisticalOutliers(nb_neighbor, std_ratio); 
    std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud = pcd -> SelectByIndex(ind);
    //std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = pcd -> SelectByIndex(ind, true);
    return inlier_cloud;
}

auto ROIfiltering(const Eigen::Vector3d &min_bound, const Eigen::Vector3d &max_bound){
    auto box = open3d::geometry::AxisAlignedBoundingBox(min_bound, max_bound);
    return box;
}

auto DBSCANclustering(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                      const double &eps = 0.3,
                      const double &min_points = 10){
    // [Group local point cloud clusters together]
    std::vector<int>labels = pcd -> ClusterDBSCAN(eps, min_points, true);
    int max_label = *max_element(labels.begin(),labels.end());
    open3d::utility::LogInfo("point cloud has {:d} clusters.", max_label+1);
    std::vector<Eigen::Vector3d> vColor; // Define color vector
	vColor.reserve(1 + max_label);
    // Generate random color
	for (int i = 0; i <= max_label; i++){
		double R = (rand()%(200-0)+0)/255.0;
		double G = (rand()%(128-100)+100)/255.0;
		double B = (rand()%(250-25)+25)/255.0;
		Eigen::Vector3d color = {R, G, B};
		vColor.push_back(color);
    }
    // Color each point cloud cluster
	std::vector<Eigen::Vector3d> pcColor;
	pcColor.reserve(labels.size());
    std::cout << "Labels Size: " << labels.size() << std::endl;
    for (int i = 0; i < labels.size(); i++){
		int label = labels[i];
		if (label <0){
			Eigen::Vector3d c = {0,0,0}; // noise=-1 set color to black
			pcColor.push_back(c);
		}else{
			Eigen::Vector3d c = vColor[label];
			pcColor.push_back(c);
		}
	}
    // Separate clustering PCD
    std::vector<std::vector<size_t>> indices;
    std::vector<size_t> indice_temp;
    //indices.reserve(max_label);
    for(int j = 0; j < max_label; j++){
        indice_temp.clear();
        for(int i = 0; i < labels.size(); i++){
            if(labels[i] == j){
                indice_temp.push_back(i);
            }
        }
        indices.push_back(indice_temp);
    }
    pcd -> colors_ = pcColor; // Paint point cloud
    return std::make_tuple(pcd, indices);
}

auto drawBoundingBox(const std::shared_ptr<open3d::geometry::PointCloud> &PCD,
                     const std::vector<std::vector<size_t>> &indices){
    std::vector<std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>> boxes;
    std::cout << "bounding indices: " << indices.size() << std::endl;
    for(int i = 0; i < indices.size(); i++){
        std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> object_box(new open3d::geometry::AxisAlignedBoundingBox);
        auto object_temp = PCD -> SelectByIndex(indices[i]);
        *object_box = object_temp -> GetAxisAlignedBoundingBox(); // Get bounding box
        object_box -> color_ = Eigen::Vector3d(1, 0, 0);
        boxes.push_back(object_box);
    }
    return boxes;
}

int main(){
    open3d::visualization::Visualizer visualizer;
    std::shared_ptr<open3d::geometry::PointCloud> pcd;
    pcd = open3d::io::CreatePointCloudFromFile("../multiway_registration.pcd");
    //pcd = open3d::io::CreatePointCloudFromFile("../data/0.pcd");
    open3d::utility::LogInfo("Load point cloud ...");
    
    //----- Crop outlier points -----//
    auto inlier_cloud = StaticalOutlierRemoval(pcd, 50, 2.0);
    open3d::utility::LogInfo("Outlier Removal ...");
    auto pcd_down = inlier_cloud -> VoxelDownSample(0.01);
    //open3d::visualization::DrawGeometries({pcd_down},"DownSampling PCD", 1600, 900);

    //----- Plane Segmentation -----//
    open3d::utility::LogInfo("Plane Segmentation...");
    auto PlaneIndex = plane_segmentation(pcd_down, 0.2, 3, 150);
    std::shared_ptr<open3d::geometry::PointCloud> plane = pcd_down -> SelectByIndex(PlaneIndex, false);
	const Eigen::Vector3d colorIn = {0, 1, 0}; // Paint the street green
	plane -> PaintUniformColor(colorIn);
	std::shared_ptr<open3d::geometry::PointCloud> outPCD = pcd_down -> SelectByIndex(PlaneIndex, true);
	const Eigen::Vector3d colorOut = {0, 0, 0};
	outPCD -> PaintUniformColor(colorOut);
    //open3d::visualization::DrawGeometries({plane, outPC}, "Plane Segmentation", 1600, 900);

    //----- ROI Filtering -----//
    auto axis = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5.0);
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> ROI_box (new open3d::geometry::AxisAlignedBoundingBox);
    *ROI_box = open3d::geometry::AxisAlignedBoundingBox(Eigen::Vector3d(-50.0, -5.0, -2.0), Eigen::Vector3d(50.0, 7.0, 10.0));
    auto ROI = outPCD -> Crop(*ROI_box);
    ROI = StaticalOutlierRemoval(ROI, 50, 1.0);
    //open3d::visualization::DrawGeometries({ROI, axis, box}, "Clustering PCD", 1600, 900);

    //----- Clustering -----//
    std::cout << '\n' << "DBSCAN Clustering..." << std::endl;
    auto ROI_inlier = StaticalOutlierRemoval(ROI, 40, 1.0);
    std::shared_ptr<open3d::geometry::PointCloud> PCD_Clustering;
    std::vector<std::vector<size_t>> PCDindices;
    std::tie(PCD_Clustering, PCDindices) = DBSCANclustering(ROI_inlier, 0.5, 60);

    //----- Bounding Box -----//
    auto ObjectBoxes = drawBoundingBox(PCD_Clustering, PCDindices);
    outPCD -> VoxelDownSample(0.7);
    //open3d::visualization::DrawGeometries({PCD_Clustering, plane, axis, ROI_box}, "Clustering PCD", 1600, 900);
    visualizer.CreateVisualizerWindow("Test", 1600, 900);
    //outPCD
    visualizer.AddGeometry(outPCD);
    for(int i=0; i < ObjectBoxes.size(); i++){
        visualizer.AddGeometry(ObjectBoxes[i]);
    }
    visualizer.AddGeometry(PCD_Clustering);
    visualizer.AddGeometry(plane);
    visualizer.AddGeometry(axis);
    visualizer.AddGeometry(ROI_box);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
    return 0;
}