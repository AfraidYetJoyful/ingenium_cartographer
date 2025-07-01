'''
Main script that smooths a .ply point cloud by using relaxation. Takes a very long time for large point clouds like we get from the liDAR, needs to be optimized.
'''
import numpy as np # for arrays
from scipy.spatial import KDTree #for finding nearby points
import open3d as o3d  

def smooth_point_cloud_towards_center(pcd, radius=0.1, smoothing_factor=0.05, step_size=1):
    """
    Moves each neighbor point in the radius closer to the main point.

    Parameters:
        pcd (open3d.geometry.PointCloud): The input point cloud.
        radius: The radius 
        smoothing_factor: Fraction of the way to move each neighbor toward the query point.
        step_size: Skip every n points to reduce processing time

    Returns:
        open3d.geometry.PointCloud: The smoothed point cloud.
    """
    points = np.asarray(pcd.points)
    tree = KDTree(points)
    
    # Copy so we don't modify the original until we're done
    smoothed_points = np.copy(points)

    # for every point around it, if it has more than one neighbor, a vector is created between the points 
    # and normalized, then multiplied by a scalar (smoothing factor) to get the distance and direction to move.
    # only runs for every (step_size) points
    for i in range(0, len(points), step_size):
        point = points[i] 
        idx = tree.query_ball_point(point, radius)
        if len(idx) >= 1:
            for idx_neighbor in idx:
                neighbor_position = smoothed_points[idx_neighbor]
                direction = point - neighbor_position
                move_distance = np.linalg.norm(direction)
                direction_normalized = direction / move_distance if move_distance != 0 else direction
                smoothed_points[idx_neighbor] = neighbor_position + direction_normalized * smoothing_factor

    smoothed_pcd = o3d.geometry.PointCloud()
    smoothed_pcd.points = o3d.utility.Vector3dVector(smoothed_points)

    return smoothed_pcd
	
	
	
	
	
	
	
	

# names point cloud pcd
pcd = o3d.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\example.ply")  # replace with path for whatever ply you're smoothing

# Calls smoothing function
smoothed_pcd = smooth_point_cloud_towards_center(pcd, radius=10, smoothing_factor=0.3, step_size=1)

# Save the result 
o3d.io.write_point_cloud("smoothed_output.ply", smoothed_pcd)

# Visualize
o3d.visualization.draw_geometries([smoothed_pcd])