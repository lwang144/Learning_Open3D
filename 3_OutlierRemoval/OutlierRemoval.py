import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

import open3d_tutorial as o3dtut

def display_inlier_outlier(cloud, ind):
    inlier_cloud = cloud.select_by_index(ind)
    outlier_cloud = cloud.select_by_index(ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
                                      zoom=0.3412,
                                      front=[0.4257, -0.2125, -0.8795],
                                      lookat=[2.6172, 2.0475, 1.532],
                                      up=[-0.0694, -0.9768, 0.2024])

##----- Prepare Input data -----##
print("Load a ply point cloud, print it, and render it")
pcd = o3d.io.read_point_cloud("../test_data/ICP/cloud_bin_2.pcd")

print("Downsample the point cloud with a voxel of 0.02")
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.02)

##----- Statistical outlier removal -----##
print("Statistical oulier removal")
# removes points that are further away from their neighbors compared to the average for the point cloud.
# [nb_neighbors]: specifies how many neighbors are taken into account in order to calculate the average distance for a given point.
# [std_ratio]: The lower this number the more aggressive the filter will be.
cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
display_inlier_outlier(voxel_down_pcd, ind)


##----- Radius outlier removal -----##
print("Radius oulier removal")
# removes points that have few neighbors in a given sphere around them.
# [nb_points]: pick the minimum amount of points that the sphere should contain.
# [radius]: defines the radius of the sphere that will be used for counting the neighbors.
cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=16, radius=0.05)
display_inlier_outlier(voxel_down_pcd, ind)

