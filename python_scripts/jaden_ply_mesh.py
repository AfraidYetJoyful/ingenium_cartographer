'''
Creates a mesh for a .ply point cloud
'''
import open3d as o3d
import numpy as np

file_path =  r"C:\Users\jtau4\Documents\lidar(relaxation)\Barrows Lower Half.ply"
# Load the PLY file
pcd = o3d.io.read_point_cloud(file_path)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])

def point_cloud_to_mesh_ball_pivoting(pcd, radii=[0.005, 0.01, 0.02]):
    # Estimate normals 
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(k=30)

    # Create a mesh using Ball Pivoting
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radii = [avg_dist * factor for factor in [1.5, 2, 2.5]]
    
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(radii))

    # Compute vertex normals
    mesh.compute_vertex_normals()

    return mesh

# Convert to mesh using Ball Pivoting
mesh = point_cloud_to_mesh_ball_pivoting(pcd)

# Save the mesh
o3d.io.write_triangle_mesh("smoothed_mesh_ball_pivoting.ply", mesh)

# Visualize
o3d.visualization.draw_geometries([mesh], window_name="Ball Pivoting Mesh")