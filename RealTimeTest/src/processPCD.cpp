#include "processPCD.h"
/* Point cloud processing using Open3D */

std::shared_ptr<open3d::geometry::PointCloud> 
loadPCD(const std::vector<std::string> &filePaths, 
        const int16_t &NUM){
    // Load .pcd file
    //std::string file = filePath + std::to_string(NUM) + ".pcd";
    std::shared_ptr<open3d::geometry::PointCloud> pcd;
    pcd = open3d::io::CreatePointCloudFromFile(filePaths[NUM]);
    std::cout << "Load file: [" << filePaths[NUM] << "]." << std::endl;
    return pcd;
}

auto
OutlierRemoval( const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                const size_t &nb_neighbor = 20, 
                const double &std_ratio = 2.0){
    std::shared_ptr<open3d::geometry::PointCloud> cl;
    std::vector<size_t> ind;
    // nb_neighbors: Specifies how many neighbors are taken into account in order to 
    //    calculate the average distance for a given point.
    // std_ratio: The lower this number the more aggressive the filter will be.
    std::tie(cl, ind) = pcd -> RemoveStatisticalOutliers(nb_neighbor, std_ratio); 
    std::shared_ptr<open3d::geometry::PointCloud> inlier_cloud  = pcd -> SelectByIndex(ind);
    //std::shared_ptr<open3d::geometry::PointCloud> outlier_cloud = pcd -> SelectByIndex(ind, true);
    return inlier_cloud;
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::pipelines::registration::Feature>> 
preProcess(const open3d::geometry::PointCloud &pcd,
                            const double &voxel_size){
    //open3d::utility::LogInfo("Preprocess :: Downsampling");
    std::shared_ptr<open3d::geometry::PointCloud> pcd_temp(new open3d::geometry::PointCloud);
    *pcd_temp = pcd;
    auto pcd_down = pcd_temp -> VoxelDownSample(voxel_size);
    pcd_down = OutlierRemoval(pcd_down, 30, 2.0);
    //open3d::utility::LogInfo("Preprocess :: Estimate normal");
    double radius_normal = voxel_size * 2;
    pcd_down -> EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(radius_normal, 30));
    double radius_feature = voxel_size * 5;
    //open3d::utility::LogInfo("Preprocess :: Compute FPFH Features");
    auto pcd_fpfh = open3d::pipelines::registration::ComputeFPFHFeature(*pcd_down,
                        open3d::geometry::KDTreeSearchParamHybrid(radius_feature, 100));
    return std::make_tuple(pcd_down, pcd_fpfh);
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>> 
ROICrop(const std::shared_ptr<open3d::geometry::PointCloud> &PCD,
        const Eigen::Vector3d &minBound, 
        const Eigen::Vector3d &maxBound){
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> ROI_box (new open3d::geometry::AxisAlignedBoundingBox);
    *ROI_box = open3d::geometry::AxisAlignedBoundingBox(minBound, maxBound);
    std::shared_ptr<open3d::geometry::PointCloud> ROI = PCD -> Crop(*ROI_box);
    return std::make_tuple(ROI, ROI_box);
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::shared_ptr<open3d::geometry::PointCloud>, std::vector<size_t>>
planeSegmentation( const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                                        const double &distance_threshold = 0.2, 
                                        const int &ransac_n = 3,
                                        const int &num_iterations = 100){
    std::tuple<Eigen::Vector4d, std::vector<size_t>> vRes = 
                    pcd -> SegmentPlane(distance_threshold, ransac_n, num_iterations); // Return plane model and inliers
    // [a b c d] plane model
	Eigen::Vector4d para = std::get<0>(vRes);
    //open3d::utility::LogInfo("Road Segmentation");
    //std::cout << "Plane equation: " << para[0] << "x + " << para[1] << "y + " << para[2] << "z + " << para[3] << " = 0" << std::endl;
    // Inliers
    std::vector<size_t> selectedIndex = std::get<1>(vRes);
    return std::make_tuple(pcd -> SelectByIndex(selectedIndex, false), pcd -> SelectByIndex(selectedIndex, true), selectedIndex);
}

std::tuple<std::shared_ptr<open3d::geometry::PointCloud>, std::vector<std::vector<size_t>>>
DBSCANclustering(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
                      const double &eps = 0.3,
                      const double &min_points = 10){
    // [Group local point cloud clusters together]
    std::vector<int>labels = pcd -> ClusterDBSCAN(eps, min_points, false);
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

std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>>
objectOrientedBoundingBox(const std::shared_ptr<open3d::geometry::PointCloud> &PCD,
                     const std::vector<std::vector<size_t>> &indices){
    std::vector<std::shared_ptr<open3d::geometry::OrientedBoundingBox>> boxes;
    std::cout << "bounding indices: " << indices.size() << std::endl;
    for(int i = 0; i < indices.size(); i++){
        std::shared_ptr<open3d::geometry::OrientedBoundingBox> object_box(new open3d::geometry::OrientedBoundingBox);
        auto object_temp = PCD -> SelectByIndex(indices[i]);
        *object_box = object_temp -> GetOrientedBoundingBox(); // Get bounding box
        object_box -> color_ = Eigen::Vector3d(1, 0, 0);
        boxes.push_back(object_box);
    }
    return boxes;
}

std::vector<std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>>
objectAxisAlignedBoundingBox(const std::shared_ptr<open3d::geometry::PointCloud> &PCD,
                     const std::vector<std::vector<size_t>> &indices){
    std::vector<std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox>> boxes;
    std::cout << "bounding indices: " << indices.size() << std::endl;
    for(int i = 0; i < indices.size(); i++){
        std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> object_box(new open3d::geometry::AxisAlignedBoundingBox);
        auto object_temp = PCD -> SelectByIndex(indices[i]);
        *object_box = object_temp -> GetAxisAlignedBoundingBox(); // Get bounding box
        auto mmm = object_box -> max_bound_;
        //std::cout << "[" << mmm[0] << ", " << mmm[1] << ", " << mmm[2] << "]" << std::endl;
        if((object_box -> Volume() < 15) && (mmm[2] < 1)){
            object_box -> color_ = Eigen::Vector3d(1, 0, 0);
            boxes.push_back(object_box);
        }
    }
    return boxes;
}