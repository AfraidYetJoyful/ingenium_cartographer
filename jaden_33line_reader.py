'''
reads first 33 lines of a ply file
'''
import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\barrows_pointcloud.bag_point_cloud.ply")
points = np.asarray(pcd.points)

print(points[:33)  # prints the first 33 [x, y, z] coordinates