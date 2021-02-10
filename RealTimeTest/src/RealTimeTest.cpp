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
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Test", 1600, 900);
    visualizer.GetRenderOption().point_size_ = 2;
    
    // Load all files in the file path
    std::string folderPath = "../data/data_2/";
    int16_t fileNum;
    std::vector<std::string> filePaths;
    std::tie(filePaths, fileNum) = fileSystem(folderPath);
    auto axis = open3d::geometry::TriangleMesh::CreateCoordinateFrame(5.0);
    
    // Loop through all files
    int16_t NUM = 0;
    while(NUM != fileNum){
        auto PCD = loadPCD(filePaths, NUM);
        // Filter point cloud with Region of Interest
        std::shared_ptr<open3d::geometry::PointCloud> ROI_PCD;
        std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> ROI_BOX;
        const Eigen::Vector3d minBound(-60.0, -7.0, -3.0);
        const Eigen::Vector3d maxBound(60.0, 7.0, 4.0);
        std::tie(ROI_PCD, ROI_BOX) = ROICrop(PCD, minBound, maxBound);

        // Visualize
        visualizer.AddGeometry(PCD);
        visualizer.AddGeometry(axis);
        visualizer.PollEvents();
        visualizer.ClearGeometries();
        std::this_thread::sleep_for(std::chrono::milliseconds(80));
        NUM ++;
    }
    visualizer.DestroyVisualizerWindow();
    return 0;
}