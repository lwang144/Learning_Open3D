#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>
#include <chrono>

#include <Eigen/Dense>

#include "open3d/Open3D.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"

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
    std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = pcd -> SelectByIndex(ind, true);
    return inlier_cloud;
}

auto load_point_cloud(const double &voxel_size = 0.0){
    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> PCDs;
    for(uint8_t i = 0; i < 22; i ++){
        std::string filePath = "../data/" + std::to_string(i) + ".pcd";
        std::shared_ptr<open3d::geometry::PointCloud> pcd;
        pcd = open3d::io::CreatePointCloudFromFile(filePath);
        auto pcd_down = pcd -> VoxelDownSample(voxel_size);
        auto inlier_cloud = StaticalOutlierRemoval(pcd_down, 50, 2.0);
        inlier_cloud -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.1, 30));
        PCDs.push_back(inlier_cloud);
        open3d::utility::LogInfo("cloud_bin_{:d}, down sampling finished.", i);
    }
    return PCDs;
}

auto pairwise_registration( const open3d::geometry::PointCloud &source,
                            const open3d::geometry::PointCloud &target,
                            const double max_correspondence_distance_coarse,
                            const double max_correspondence_distance_fine){
    open3d::utility::LogInfo("Apply Point-to-plane ICP");
    //----- Coarse registration -----//
    auto icp_coarse = open3d::pipelines::registration::RegistrationICP(source, target, max_correspondence_distance_coarse,
                            Eigen::MatrixXd::Identity(4,4), open3d::pipelines::registration::TransformationEstimationPointToPlane());
    //----- Fine registration -----//
    auto icp_fine   = open3d::pipelines::registration::RegistrationICP(source, target, max_correspondence_distance_fine,
                            icp_coarse.transformation_, open3d::pipelines::registration::TransformationEstimationPointToPlane());
    auto transformationICP = icp_fine.transformation_;
    auto informationICP = open3d::pipelines::registration::GetInformationMatrixFromPointClouds(source, target, 
                                        max_correspondence_distance_fine, icp_fine.transformation_);
    return std::make_tuple(transformationICP, informationICP);
}

auto full_registration( const std::vector<std::shared_ptr<open3d::geometry::PointCloud>> &PCDs,
                        const double max_correspondence_distance_coarse,
                        const double max_correspondence_distance_fine){
    auto pose_graph = open3d::pipelines::registration::PoseGraph();
    auto odomentry  = Eigen::MatrixXd::Identity(4,4);
    pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odomentry));
    for(int sourceID = 0; sourceID < PCDs.size(); sourceID ++){
        for(int targetID = sourceID + 1; targetID < PCDs.size(); targetID ++){
            Eigen::Matrix4d transformationICP; 
            Eigen::Matrix6d_u informationICP;
            std::tie(transformationICP, informationICP) = pairwise_registration(*PCDs[sourceID], *PCDs[targetID], 
                                                            max_correspondence_distance_coarse, max_correspondence_distance_fine);
            open3d::utility::LogInfo("   Build open3d::pipelines::registration::PoseGraph");
            if(targetID == sourceID + 1){ // Odometry Case
                open3d::utility::LogInfo("   PCD[{:d}, {:d}], --Odometry Edge", sourceID, targetID);
                auto odomentry_temp = transformationICP * odomentry;
                pose_graph.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(odomentry_temp.inverse()));
                pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(sourceID, targetID, 
                                                                    transformationICP, informationICP, false));
            }
            else{
                open3d::utility::LogInfo("   PCD[{:d}, {:d}], --Loop Closure Edge", sourceID, targetID);
                pose_graph.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(sourceID, targetID, 
                                                                    transformationICP, informationICP, true));
            }
        }
    }
    return pose_graph;
}

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
    for (int i = 0; i < labels.size(); i++)
	{
		int label = labels[i];
		if (label <0){
			Eigen::Vector3d c = {0,0,0}; // noise=-1 set color to black
			pcColor.push_back(c);
		}else{
			Eigen::Vector3d c = vColor[label];
			pcColor.push_back(c);
		}
	}
    // Paint point cloud
    pcd -> colors_ = pcColor;
    return pcd;
}

int main(){
    open3d::visualization::Visualizer visualizer; //Create a visualizer object
    //----- 1. Prepare Point cloud -----//
    double voxel_size = 0.1;
    std::cout << '\n' << "[ 1. Load Pointcloud: ]" << std::endl;
    auto PCDs_down = load_point_cloud(voxel_size);
    
    //----- 2. Full Registration -----//
    std::cout << '\n' << "[ 2. Full Registration: ]" << std::endl;
    double max_correspondence_distance_coarse = voxel_size * 15;
    double max_correspondence_distance_fine = voxel_size * 1.5;
    auto pose_graph = full_registration(PCDs_down, max_correspondence_distance_coarse, max_correspondence_distance_fine);

    //----- 3. Optimizing PoseGraph -----//
    std::cout << '\n' << "[ 3. Optimizing PoseGraph ...]" << std::endl;
    auto option = open3d::pipelines::registration::GlobalOptimizationOption(max_correspondence_distance_fine, 0.25, 0);
    open3d::pipelines::registration::GlobalOptimization(pose_graph, open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt(),
                                                        open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria(), option);

    //----- 4. Transform points and display -----//
    std::cout << '\n' << "[ 4. Transform points and display: ]" << std::endl;
    for(int pointID = 0; pointID < PCDs_down.size(); pointID ++){
        open3d::utility::LogInfo("PCD_{:d}.nodes.pose:", pointID);
        std::cout << "[" << pose_graph.nodes_[pointID].pose_ << "]" << std::endl;
        PCDs_down[pointID] -> Transform(pose_graph.nodes_[pointID].pose_);
    }

     //----- Point cloud Visualization -----//
    visualizer.CreateVisualizerWindow("Transformed PCD", 1600, 900);
    for(int ID = 0; ID < PCDs_down.size(); ID ++){    
        visualizer.AddGeometry(PCDs_down[ID]);
    }
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    //----- 5. Combine transformed PointClouds -----//
    std::cout << '\n' << "[ 5. Combined transform Pointcloud: ]" << std::endl;
    auto PCDs = load_point_cloud(voxel_size);
    std::shared_ptr<open3d::geometry::PointCloud> PCDs_combined(new open3d::geometry::PointCloud);
    for(int ID = 0; ID < PCDs.size(); ID ++){    
        PCDs[ID] -> Transform(pose_graph.nodes_[ID].pose_);
        *PCDs_combined += *PCDs[ID];
    }
    auto PCDs_combined_down = PCDs_combined -> VoxelDownSample(voxel_size);
    open3d::io::WritePointCloud("../multiway_registration.pcd", *PCDs_combined_down);
    open3d::visualization::DrawGeometries({PCDs_combined_down}, "Combined Multiway_registration PCD", 1600, 900);
    
    //----- 6. Outlier Removal -----//
    auto inlier_combined_cloud = StaticalOutlierRemoval(PCDs_combined_down, 50, 2.0);
    open3d::utility::LogInfo("[6. Outlier Removal: ]");

    //----- 7. Plane Segmentation -----//
    open3d::utility::LogInfo("[7. Plane Segmentation: ]");
    auto PlaneIndex = plane_segmentation(inlier_combined_cloud, 0.2, 3, 150);
    std::shared_ptr<open3d::geometry::PointCloud> plane = inlier_combined_cloud -> SelectByIndex(PlaneIndex, false);
	const Eigen::Vector3d colorIn = {0, 1, 0}; // Paint the street green
	plane -> PaintUniformColor(colorIn);
	std::shared_ptr<open3d::geometry::PointCloud> outPC = inlier_combined_cloud -> SelectByIndex(PlaneIndex, true);
	const Eigen::Vector3d colorOut = {1, 0, 0};
	outPC -> PaintUniformColor(colorOut);
    open3d::visualization::DrawGeometries({plane, outPC}, "Plane Segmentation", 1600, 900);


    //----- 7. Clustering -----//
    // std::cout << '\n' << "[ 7. DBSCAN Clustering: ]" << std::endl;
    // auto PCD_Clustering = DBSCANclustering(PCDs_combined_down);
    // open3d::visualization::DrawGeometries({PCD_Clustering}, "Combined Multiway_registration PCD", 1600, 900);
    
    return 0;
}