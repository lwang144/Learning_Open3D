# Point Cloud Outlier Removal
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

When collecting data from scanning devices, the resulting point cloud tends to contain noise and artifacts that one would like to remove.

[**C++ Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/3_OutlierRemoval/src/OutlierRemoval.cpp)

[**Python Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/3_OutlierRemoval/OutlierRemoval.py)

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
  - Statistical outlier removal
  ```
  Python: cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors, std_ratio)

  C++: std::tie(cl, ind) = cloud_ptr -> RemoveStatisticalOutliers(nb_neighbors, std_ratio);
  ```
  - Radius outlier removal
  ```
  Python: cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points, radius)

  C++: std::tie(cl, ind) = cloud_ptr -> RemoveRadiusOutliers(nb_points, radius);
  ```