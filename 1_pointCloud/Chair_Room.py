import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

#sys.path.append('..')
#import open3d_tutorial as o3dtut

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
# ilters out points. Only the chair remains.
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


# visualize the point cloud
o3d.visualization.draw_geometries([chair, boundingAligned, boundingOriented],
                                  zoom=0.5,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024],
                                  point_show_normal=False)
