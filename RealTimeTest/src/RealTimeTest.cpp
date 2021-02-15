/* \author Leo Wang */
// Custom function for pointcloud processing 
// using Open3D and Eigen

/**
 * Developer: Leo Wang
 * E-mail:    liangyu@student.chalmers.se
 * Date:      02/2020
 */
#include "processPCD.h"
#include "supportFunction.cpp"
#include "processPCD.cpp"

int main(){
    open3d::visualization::Visualizer visualizer; //Create a visualizer object
    visualizer.CreateVisualizerWindow("Test", 1600, 900);
    visualizer.GetRenderOption().point_size_ = 2;

    // Load all files in the file path
    std::string folderPath = "../data/data_1/";
    int16_t fileNum;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = fileSystem(folderPath);
    auto axis = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5.0);
    
    // Loop through all files
    int16_t NUM = 0;
    while(NUM != fileNum){
        // Load .pcd file
        auto PCD = loadPCD(filePaths, NUM);
        PCD -> PaintUniformColor(Eigen::Vector3d(0, 0, 0));

        // Preprocessing
        double voxel_size = 0.2; // Mean 5 cm for this dataset
        std::shared_ptr<open3d::geometry::PointCloud> PCD_DOWN;
        std::shared_ptr<open3d::pipelines::registration::Feature> PCD_FPFH;
        std::tie(PCD_DOWN, PCD_FPFH) = preProcess(*PCD, voxel_size); 

        // Outlier Removal
        const size_t nb_neighbor = 20;
        const double &std_ratio = 2.0;
        PCD_DOWN -> RemoveStatisticalOutliers(nb_neighbor, std_ratio); 

        // Road Segmentation
        std::shared_ptr<open3d::geometry::PointCloud> Plane;
        std::shared_ptr<open3d::geometry::PointCloud> PCD_noStreet;
        std::vector<size_t> Plane_index;
        std::tie(Plane, PCD_noStreet, Plane_index) = planeSegmentation(PCD_DOWN, 0.2, 3, 100);
        Plane -> PaintUniformColor(Eigen::Vector3d(0, 1, 0));

        // Filter point cloud with Region of Interest (ROI)
        std::shared_ptr<open3d::geometry::PointCloud> ROI_PCD;
        std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> ROI_BOX;
        const Eigen::Vector3d minBound(-40.0, -10.0, -3.0);
        const Eigen::Vector3d maxBound(40.0, 10.0, 4.0);
        std::tie(ROI_PCD, ROI_BOX) = ROICrop(PCD_noStreet, minBound, maxBound);

        // Object Clustering
        auto ROI_inlier = OutlierRemoval(ROI_PCD, 30, 1.0);
        std::shared_ptr<open3d::geometry::PointCloud> PCD_Clustering;
        std::vector<std::vector<size_t>> PCDindices;
        std::tie(PCD_Clustering, PCDindices) = DBSCANclustering(ROI_inlier, 0.5, 15);

        // Draw Bounding boxes
        auto ObjectBoxes = objectBoundingBox(PCD_Clustering, PCDindices);

        // Visualize
        for(int i=0; i < ObjectBoxes.size(); i++){
           visualizer.AddGeometry(ObjectBoxes[i]);
        }
        visualizer.AddGeometry(Plane);
        visualizer.AddGeometry(PCD_noStreet);
        visualizer.AddGeometry(ROI_BOX);
        visualizer.AddGeometry(axis);
        open3d::visualization::ViewControl &view_control = visualizer.GetViewControl();
        //view_control.SetFront(Eigen::Vector3d(0, -1, 0));
        view_control.SetZoom(0.2);
        visualizer.PollEvents();
        visualizer.ClearGeometries();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        NUM ++;
    }
    visualizer.DestroyVisualizerWindow();
    return 0;
}