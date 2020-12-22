# KDTree
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

Open3D uses FLANN to build KDTrees for fast retrieval of nearest neighbors. [FLANN Introduction](https://www.cs.ubc.ca/research/flann/uploads/FLANN/flann_manual-1.8.4.pdf)


Open3D supported nearest neighbor search method:
  1. K Nearest Neighbors Search
  2. Radius Nearest Neighbors Search
  3. RKNN Radius K Nearest Neighbors Search

[**C++ Examples**](https://github.com/LYON-WANG/Learning_Open3D/tree/master/2_KDTree/src/KDTree.cpp)

[**Python Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/2_KDTree/KDTree.py)

**RUN C++ Example:** 
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./KDTree ../../test_data/Feature/cloud_bin_0.pcd
```

## Common Geometry KDTree Function Summary:
  - Build KDTree [kdtree] from point cloud [pcd]
  ```
  Python: pcd_tree = o3d.geometry.KDTreeFlann(pcd)

  C++: open3d::geometry::KDTreeFlann kdtree;
       kdtree.SetGeometry(*pcd);
  ```
  - K Nearest Neighbors Search: Returns a list of indices of the k nearest neighbors of the anchor point.
  ```
  Python: [k, idx, _] = pcd_tree.search_knn_vector_3d(query, int knn)

  C++: kdtree.SearchKNN(query, knn, indices, distance2);
  ```
  - Radius Nearest Neighbors Search: Query all points with distances to the anchor point less than a given radius. 
  ```
  Python: [k, idx, _] = pcd_tree.search_radius_vector_3d(query, float radius)

  C++: kdtree.SearchRadius(query, float radius);
  ```
  - RKNN Radius K Nearest Neighbors Search
  ```
  Python: [k, idx, _] = pcd_tree.search_hybrid_vector_3d(query, float radius, int knn)

  C++: kdtree.SearchHybrid(query, float radius, int knn);
  ```
  - Paint 1500th point red
  ```
  Python: pcd.colors[1500] = [1, 0, 0]

  C++: cloud_ptr -> colors_[1500] = Eigen::Vector3d(1, 0, 0);
  ```