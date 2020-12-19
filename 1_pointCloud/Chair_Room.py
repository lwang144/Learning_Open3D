import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

import open3d_tutorial as o3dtut

##----- Load point cloud -----##
print("Load a ply point cloud, print it, and render it")
# Read a point cloud from file  .xyz  .xyzn  .xyzrgb   .pts   .ply   .pcd
pcd = o3d.io.read_point_cloud("../test_data/fragment.ply") 
print(pcd)
print(np.asarray(pcd.points))

##----- Voxel downsampling -----##
# Voxel Downsampling uses a regular voxel grid 
# to create a uniformly downsampled point cloud
print("Downsample the point cloud with a voxel of 0.05")
downpcd = pcd.voxel_down_sample(voxel_size = 0.05)


##----- Vertex  normal estimation -----##
# Compute normal for every point parameter[radius: searching radius, max_nn: maximum nearest neighbor]
print("Recompute the normal of the downsampled point cloud")
downpcd.estimate_normals(search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) 
# 10cm of search radius, and only considers up to 30 neighbors to save computation time
print("Print a normal vector of the 0th point")
print(downpcd.normals[0])


##----- Crop point cloud -----##
print("Load a polygon volume and use it to crop the original point cloud")
# reads a json file that specifies polygon selection area
vol = o3d.visualization.read_selection_polygon_volume("../test_data/Crop/cropped.json")
# filters out points. Only the chair remains.
chair = vol.crop_point_cloud(pcd)


##----- Paint point cloud -----##
print("Paint chair")
chair.paint_uniform_color([1,0.706,0]) # Paint yellow


##----- Point cloud distance -----##
# Compute the distance from the source point cloud [pcd] to a target point cloud [chair]
dists = pcd.compute_point_cloud_distance(chair)
dists = np.asarray(dists)
ind = np.where(dists > 0.01)[0] # Find point belong to the [chair] point cloud
pcd_without_chair = pcd.select_by_index(ind) # Remove the chair points

##----- Bounding volumes -----##
boundingAligned = chair.get_axis_aligned_bounding_box()
boundingAligned.color = (1, 0, 0)
boundingOriented = chair.get_oriented_bounding_box()
boundingOriented.color = (0, 1, 0)


##----- Convex hull -----##
#rabit = o3dtut.get_bunny_mesh().sample_points_poisson_disk(number_of_points=2000)
#hull, _ = rabit.compute_convex_hull()
#hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
#hull_ls.paint_uniform_color([1,0,0])
#o3d.visualization.draw_geometries([rabit, hull_ls])

##----- DBSCAN Clustering -----##
# Group local point cloud clusters together
with o3d.utility.VerbosityContextManager(
        o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(
        pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

##----- Plane segmentation -----##
# Find the plane with the largest support in the point cloud
pcd = o3d.io.read_point_cloud("../test_data/fragment.pcd")
# [distance_threshold: defines the maximum distance a point can have to an estimated plane to be considered an inlier]
# [ransac_n: defines the number of points that are randomly sampled to estimate a plane]
# [num_iteration: defines how often a random plane is sampled and verified]
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)

# visualize the point cloud
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                  zoom=0.5,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=False)

##----- Hidden point removal -----##
print("Convert mesh to a point cloud and estimate dimensions")
pcd = o3dtut.get_armadillo_mesh().sample_points_poisson_disk(5000)
diameter = np.linalg.norm(
    np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
o3d.visualization.draw_geometries([pcd])

print("Define parameters used for hidden_point_removal")
camera = [0, 0, diameter]
radius = diameter * 100

print("Get all points that are visible from given view point")
_, pt_map = pcd.hidden_point_removal(camera, radius)

print("Visualize result")
pcd = pcd.select_by_index(pt_map)
o3d.visualization.draw_geometries([pcd])