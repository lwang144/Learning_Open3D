#include "processPCD.h"

auto loadPCD(const std::string &filePath, const int16_t &NUM){
    std::string file = filePath + std::to_string(NUM) + ".pcd";
    std::shared_ptr<open3d::geometry::PointCloud> pcd;
    //pcd = open3d::io::CreatePointCloudFromFile("../multiway_registration.pcd");
    //pcd = open3d::io::CreatePointCloudFromFile("../data/0.pcd");
    open3d::utility::LogInfo("Load point cloud ...");
    std::cout << file << std::endl;
}

std::vector<size_t> plane_segmentation(const std::shared_ptr<open3d::geometry::PointCloud> &pcd,
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

std::shared_ptr<open3d::geometry::PointCloud> StaticalOutlierRemoval(const std::shared_ptr<open3d::geometry::PointCloud> &pcd, 
            const size_t &nb_neighbor = 20, const double &std_ratio = 2.0){
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
