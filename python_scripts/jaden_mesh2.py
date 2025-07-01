import open3d as o3d

# Load your point cloud
pcd = o3d.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\barrows_pointcloud.bag_point_cloud.ply")

# Quick meshing (you can use Ball Pivoting or Poisson here)
# Example: Poisson (order = smoothness)
mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8)  # depth can be tuned

# Optional: crop mesh to remove crazy floating pieces (not strictly needed)
bbox = pcd.get_axis_aligned_bounding_box()
mesh = mesh.crop(bbox)

# --- Apply Laplacian smoothing ---
# Smooth the mesh without changing topology
mesh = mesh.filter_smooth_laplacian(number_of_iterations=5)  # 5 iterations is pretty normal
mesh.compute_vertex_normals()

# Save the smoothed mesh
o3d.io.write_triangle_mesh("smoothed_mesh_laplacian.ply", mesh)

# Visualize
o3d.visualization.draw_geometries([mesh])