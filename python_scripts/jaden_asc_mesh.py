'''
Smooths and creates a mesh for a .asc point cloud (Probably not useful)
'''
import numpy as np
import open3d as o3d
import pandas as pd
import laspy

# Load point cloud with positions and normals
def load_point_cloud(file_path, delimiter=" "):
    data = pd.read_csv(file_path, delimiter=delimiter, header=None)
    
    if data.shape[1] < 6:
        raise ValueError("File must have at least 6 columns (X, Y, Z, Nx, Ny, Nz).")

    # Extract X, Y, Z coordinates
    points = data.iloc[:, :3].to_numpy()
    
    # Extract normal vectors
    normals = data.iloc[:, 3:6].to_numpy()

    # Convert to Open3D format
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.normals = o3d.utility.Vector3dVector(normals)

    return pcd

# Example usage
file_path = "exampledata.asc"  # Change this to your data file name
pcd = load_point_cloud(file_path, delimiter=" ")

# Visualize the raw point cloud
# o3d.visualization.draw_geometries([pcd])

# Save to PLY format for future use
o3d.io.write_point_cloud("converted.ply", pcd)

from scipy.spatial import KDTree

def smooth_point_cloud_radius(pcd, radius=0.05, alpha=0.5):
    points = np.asarray(pcd.points)
    tree = KDTree(points)
    smoothed_points = np.copy(points)

    for i, point in enumerate(points):
        idx = tree.query_ball_point(point, radius)  # Find all points in radius
        if len(idx) > 1:  # Ensure we have enough neighbors
            neighbor_points = points[idx]  # Get their positions
            avg_position = np.mean(neighbor_points, axis=0)  # Compute average position
            
            # Move point slightly toward the average position
            smoothed_points[i] = (1 - alpha) * point + alpha * avg_position

    # Update point cloud
    pcd.points = o3d.utility.Vector3dVector(smoothed_points)
    return pcd

# Load and smooth the point cloud
file_path = "exampledata.asc"  # Update with your file name
pcd = load_point_cloud(file_path, delimiter=" ")

# Apply smoothing
smoothed_pcd = smooth_point_cloud_radius(pcd, radius=0.05, alpha=0.5)

# Visualize before and after
# o3d.visualization.draw_geometries([pcd], window_name="Original")
o3d.visualization.draw_geometries([smoothed_pcd], window_name="Smoothed")

# Save smoothed point cloud
o3d.io.write_point_cloud("smoothed.ply", smoothed_pcd)


#ball pivoting mesh creating method
def point_cloud_to_mesh_ball_pivoting(pcd, radii=[0.005, 0.01, 0.02]):
    # Estimate normals (if needed)
    pcd.estimate_normals()

    # Create a mesh using Ball Pivoting
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radii = [avg_dist * factor for factor in [1.5, 2, 2.5]]
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    # Compute vertex normals
    mesh.compute_vertex_normals()

    return mesh

# Convert to mesh using Ball Pivoting
mesh = point_cloud_to_mesh_ball_pivoting(smoothed_pcd)

# Save the mesh
o3d.io.write_triangle_mesh("smoothed_mesh_ball_pivoting.ply", mesh)

# Visualize
o3d.visualization.draw_geometries([mesh], window_name="Ball Pivoting Mesh")


#Poisson mesh creating method (doesn't seem to work very well) 
'''
def point_cloud_to_mesh_poisson(pcd, depth=9):
    # Apply Poisson surface reconstruction
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    
    # Compute vertex normals for better visualization
    mesh.compute_vertex_normals()

    return mesh

# Load your smoothed point cloud
pcd = o3d.io.read_point_cloud("smoothed.ply")

# Convert to mesh
mesh = point_cloud_to_mesh_poisson(pcd, depth=9)

# Save the mesh
o3d.io.write_triangle_mesh("smoothed_mesh.ply", mesh)

# Visualize
o3d.visualization.draw_geometries([mesh], window_name="Poisson Mesh")
'''
