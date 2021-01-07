#include <iostream>
#include <memory>
#include <thread>
#include <random>
#include <cstdlib>

#include <Eigen/Dense>

#include "open3d/Open3D.h"

// To get a triangle mesh from this unstructured input we 
// need to perform surface reconstruction.
// In the literature there exists a couple of methods and 
// Open3D currently implements the following:

// Alpha shapes [Edelsbrunner1983]
// Ball pivoting [Bernardini1999]
// Poisson surface reconstruction [Kazhdan2006]
int main(int argc, char* argv[]) 
{
    auto mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    mesh = open3d::io::CreateMeshFromFile(argv[1]);
    open3d::utility::LogInfo("Mesh has {:d} vertices, {:d} triangles.",
                        mesh -> vertices_.size(), mesh -> triangles_.size());
    mesh -> RemoveDuplicatedVertices();
    mesh -> RemoveDuplicatedTriangles();
    mesh -> RemoveDegenerateTriangles();
    mesh -> RemoveUnreferencedVertices();
    open3d::utility::LogInfo(
                "After purge vertices, Mesh1 has {:d} vertices, {:d} "
                "triangles.",
                mesh -> vertices_.size(), mesh -> triangles_.size());

    // Sample points from the mesh, where each point has approximately 
    // the same distance to the neighbouring points. 
    auto pcd = mesh -> SamplePointsPoissonDisk(750); // Number of points
    open3d::visualization::DrawGeometries({pcd});

    //----- Alpha Shape -----//
    double alpha = 0.03;
    std::cout << "alpha= " << alpha << std::endl;
    
    auto mesh1 = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*pcd, alpha);
    mesh1 -> ComputeVertexNormals();
    open3d::visualization::DrawGeometries({mesh1}, "Alpha Shape", 640,480,50,50,false,false,true);


    //----- Ball Pivoting -----//
    // This algorithm assumes the Point cloud has normals
    mesh -> ComputeVertexNormals();  // ReconstructBallPivoting requires normals
    auto pcd1 = mesh -> SamplePointsPoissonDisk(3000); // Number of points
    open3d::visualization::DrawGeometries({pcd1});
    
    // Define the radii of the ball that are used for surface reconstruction 
    const std::vector<double> radii={0.005, 0.01, 0.02, 0.04};
    auto rec_mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pcd1,radii);
    open3d::visualization::DrawGeometries({pcd1, rec_mesh}, "Ball Pivoting");


    //----- Poisson Surface Reconstruction -----//
    
    return 0;
}