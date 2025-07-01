'''
Visualizes cloud with warmer colors representing higher intensity 
'''
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

pcd = o3d.t.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\Tor_.ply")
points = pcd.point["positions"].numpy()

# Check if intensity exists
if "intensity" in pcd.point:
    intensity = pcd.point["intensity"].numpy().flatten()
    print("Intensity loaded")

    # Normalize intensity
    intensity_norm = (intensity - intensity.min()) / (intensity.max() - intensity.min())
    colors = plt.cm.jet(intensity_norm)[:, :3] 

    # Assign color and convert to legacy format for visualization
    pcd.point["colors"] = o3d.core.Tensor(colors, dtype=o3d.core.Dtype.Float32)
    legacy_pcd = pcd.to_legacy()

    o3d.visualization.draw_geometries([legacy_pcd])

else:
    print("Intensity not found in point cloud.")