# Learning_Open3D
The exercise included are based on the tutorial from Open3D.

Python Function Summary:
- Read a point cloud from file  .xyz  .xyzn  .xyzrgb   .pts   .ply   .pcd
  xxx = 03d.io.read_point_cloud("file path") 
- Voxel Downsampling uses a regular voxel grid
  xxx = xxx.voxel_down_sample(voxel_size = float)

- Compute normal for every point parameter[radius: searching radius, max_nn: maximum nearest neighbor]
  xxx.estimate_normals()
