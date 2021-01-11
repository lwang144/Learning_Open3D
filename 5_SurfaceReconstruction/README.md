# Surface Reconstruction
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

[**C++ Examples**](https://github.com/LYON-WANG/Learning_Open3D/tree/master/5_SurfaceReconstruction/src)

[**Python Poisson Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/5_SurfaceReconstruction/SurfaceReconstruction.py)

**RUN C++ Example:** 
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./PoissonReconstruction ../../test_data/eagle.ply 
  ./SurfaceReconstruction ../../test_data/Bunny.ply
```

## Common Function Summary:
  - Alpha shapes surface reconstruction
  ```
  Python: create_from_point_cloud_alpha_shape(pcd, alpha)

  C++: auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudAlphaShape(*pcd, alpha);
  ```
  - Ball pivoting surface reconstruction
  ```
  Python: create_from_point_cloud_ball_pivoting(pcd, radii)

  C++: auto mesh = open3d::geometry::TriangleMesh::CreateFromPointCloudBallPivoting(*pcd,radii);
  ```
  - Ball pivoting surface reconstruction
  ```
  Python: create_from_point_cloud_poisson(pcd, depth)

  C++: std::tie(mesh, dentities)= open3d::geometry::TriangleMesh::CreateFromPointCloudPoisson(*pcd, depth);
    
  ```