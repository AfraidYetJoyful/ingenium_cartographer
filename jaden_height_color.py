'''
visualizes cloud with warmer colors representing z direction hieght
'''
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Load the point cloud
pcd = o3d.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\barrows_pointcloud.bag_point_cloud.ply")
points = np.asarray(pcd.points)

# Normalize Z values between 0 and 1
z_vals = points[:, 2]
z_min = z_vals.min()
z_max = z_vals.max()
z_norm = (z_vals - z_min) / (z_max - z_min)

# Use matplotlib colormap to assign RGB colors
colors = plt.cm.viridis(z_norm)[:, :3]  # Drop alpha channel
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize
o3d.visualization.draw_geometries([pcd])