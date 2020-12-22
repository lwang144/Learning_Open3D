import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

import open3d_tutorial as o3dtut

##----- Build KDTree from point cloud -----##
print("Testing KDTree in Open3D ...")
print("Load a point cloud and paint it gray")
pcd = o3d.io.read_point_cloud("../test_data/Feature/cloud_bin_0.pcd")
pcd.paint_uniform_color([0.5, 0.5, 0.5])
pcd_tree = o3d.geometry.KDTreeFlann(pcd)

##----- Find neighbouring points -----##
# pick the 1500th point as the anchor point and paint it red.
print("Paint the 1500th point red.")
pcd.colors[1500] = [1, 0, 0]

##----- Using search_knn_vector_3d -----##
# returns a list of indices of the k nearest neighbors of the anchor point.
# These neighboring points are painted with blue color. 
# skip the first index since it is the anchor point itself.
print("Find its 200 nearest neighbors, and paint them blue.")
[k, idx, _] = pcd_tree.search_knn_vector_3d(pcd.points[1500], 200)
np.asarray(pcd.colors)[idx[1:], :] = [0, 0, 1]

##----- Using search_radius_vector_3d -----##
# to query all points with distances to the anchor point less than a given radius.
# paint these points with a green color.
print("Find its neighbors with distance less than 0.2, and paint them green.")
[k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[1500], 0.2)
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

##----- Using search_hybrid_vector_3d -----##
# Radius K Neasrest Neighbors Search (Hybrid search)
[k, idx, _] = pcd_tree.search_hybrid_vector_3d(pcd.points[1500], 0.2, 50)
np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0.8]

print("Visualize the point cloud.")
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.5599,
                                  front=[-0.4958, 0.8229, 0.2773],
                                  lookat=[2.1126, 1.0163, -1.8543],
                                  up=[0.1007, -0.2626, 0.9596],
                                  point_show_normal=False)