# ISS keypoints
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

When collecting data from scanning devices, the resulting point cloud tends to contain noise and artifacts that one would like to remove.

[**C++ Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/7_ISSKeypoint/src/ISSKeypoint.cpp)

[**Python Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/7_ISSKeypoint/ISSKeypoint.py)

**RUN C++ Example:** 
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./ISSKeypoint ../../test_data/Armadillo.ply 
```

## Common Function Summary:
  - ISS keypoint detection
  ```
  Python: keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd)

  C++: keypoints = open3d::geometry::keypoint::ComputeISSKeypoints(*pcd);
  ```
  