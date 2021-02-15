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
    open3d::visualization::visualizer::O3DVisualizer visualizer{"Test", 640, 320};
    //visualizer.SetPointSize(2);
    
    // Load all files in the file path
    std::string folderPath = "../data/data_2/";
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
        double voxel_size = 0.05; // Mean 5 cm for this dataset
        std::shared_ptr<open3d::geometry::PointCloud> PCD_DOWN;
        std::shared_ptr<open3d::pipelines::registration::Feature> PCD_FPFH;
        std::tie(PCD_DOWN, PCD_FPFH) = preProcess(*PCD, voxel_size); 

        // Outlier Removal
        const size_t nb_neighbor = 20;
        const double &std_ratio = 2.0;
        PCD_DOWN -> RemoveStatisticalOutliers(nb_neighbor, std_ratio); 

        // Filter point cloud with Region of Interest
        std::shared_ptr<open3d::geometry::PointCloud> ROI_PCD;
        std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> ROI_BOX;
        const Eigen::Vector3d minBound(-50.0, -7.0, -3.0);
        const Eigen::Vector3d maxBound(50.0, 7.0, 4.0);
        std::tie(ROI_PCD, ROI_BOX) = ROICrop(PCD_DOWN, minBound, maxBound);

        // Road Segmentation
        std::shared_ptr<open3d::geometry::PointCloud> Plane;
        std::shared_ptr<open3d::geometry::PointCloud> PCD_noStreet;
        std::vector<size_t> Plane_index;
        std::tie(Plane, PCD_noStreet, Plane_index) = planeSegmentation(PCD_DOWN, 0.2, 3, 150);
        Plane -> PaintUniformColor(Eigen::Vector3d(0, 1, 0));


        // Visualize
        //visualizer.AddGeometry("Test", Plane);
        // visualizer.AddGeometry(Plane);
        // visualizer.AddGeometry(axis);
        // auto params = open3d::camera::PinholeCameraParameters();
        // open3d::io::ReadIJsonConvertible("../view_point.json", params);
        // auto view_control = visualizer.GetViewControl();
        // view_control.ConvertFromPinholeCameraParameters(params, true);
        // visualizer.PollEvents();
        // visualizer.ClearGeometries();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        NUM ++;
    }
    //visualizer.DestroyVisualizerWindow();
    return 0;
}