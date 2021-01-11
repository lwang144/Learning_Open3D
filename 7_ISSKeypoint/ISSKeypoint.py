import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys
import time
import open3d_tutorial as o3dtut

def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.001)
        sphere.translate(keypoint)
        spheres += sphere
    spheres.paint_uniform_color([1.0, 0.75, 0.0])
    return spheres

# Compute ISS Keypoints on Armadillo
mesh = o3dtut.get_armadillo_mesh()
pcd = o3d.geometry.PointCloud()
pcd.points = mesh.vertices

tic = time.time()
# Compute ISS keypoints on Armadillo
keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd)
toc = 1000 * (time.time() - tic)
print("ISS Computation took {:.0f} [ms]".format(toc))

mesh.compute_vertex_normals()
mesh.paint_uniform_color([0.5, 0.5, 0.5])
keypoints.paint_uniform_color([1.0, 0.75, 0.0])
o3d.visualization.draw_geometries([keypoints, mesh], front=[0, 0, -1.0])

# Compute ISS Keypoints on Standford Bunny, changing the default parameters
mesh1 = o3dtut.get_armadillo_mesh()
pcd1 = o3d.geometry.PointCloud()
pcd1.points = mesh1.vertices

tic1 = time.time()
# Compute ISS keypoints on Armadillo
keypoints_bunny = o3d.geometry.keypoint.compute_iss_keypoints(pcd1,
                                                        salient_radius=0.005,
                                                        non_max_radius=0.005,
                                                        gamma_21=0.5,
                                                        gamma_32=0.5)
toc1 = 1000 * (time.time() - tic1)
print("ISS Computation took {:.0f} [ms]".format(toc1))

mesh1.compute_vertex_normals()
mesh1.paint_uniform_color([0.5, 0.5, 0.5])
o3d.visualization.draw_geometries([keypoints_to_spheres(keypoints_bunny), mesh1])
