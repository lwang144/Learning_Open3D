# Transformation
The examples included above are based on the tutorial from Open3D using both Python and C++. Hope this document can help you! 

When collecting data from scanning devices, the resulting point cloud tends to contain noise and artifacts that one would like to remove.

[**C++ Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/6_Transformation/src/Transformation.cpp)

[**Python Poisson Examples**](https://github.com/LYON-WANG/Learning_Open3D/blob/master/6_Transformation/transformation.py)

**RUN C++ Example:** 
```
  mkdir build
  cd build
  cmake ..
  make
  cd ../bin
  ./Transformation
```

## Common Function Summary:
  - Translate
  ```
  Python: mesh.translate((2,2,2), relative)

  C++: mesh -> Translate(Eigen::Vector3d(2, 2, 2));
  ```
  - Rotation
  ```
  Python: R = mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 4))
          mesh_r.rotate(R, center=(0, 0, 0))

  C++: auto R = mesh_r -> GetRotationMatrixFromXYZ(Eigen::Vector3d(PI/2, 0, PI/4));
       mesh_r ->Rotate(R, Eigen::Vector3d(0, 0, 0));
  ```
  - Scale
  ```
  Python: mesh.scale(0.5, center=(0,0,0))

  C++: mesh_s -> Scale(0.5, Eigen::Vector3d(0, 0, 0)); // Scale ratio & relative 
  ```
  - General Transformation
  ```
  Python: T = np.eye(4)
          T[:3, :3] = mesh.get_rotation_matrix_from_xyz((0, np.pi / 3, np.pi / 2))
          T[0, 3] = 1
          T[1, 3] = 1.3mesh.transform(T)

  C++:  mesh -> Transform(T);
  ```