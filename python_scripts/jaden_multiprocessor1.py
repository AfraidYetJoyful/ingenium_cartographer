'''
This Python script loads a LiDAR point cloud from a .ply file, then smooths the points by adjusting each point's position slightly toward the center of its local neighborhood. To speed up processing, the script splits the point cloud into chunks and smooths each chunk in parallel using multiple CPU cores (via Python’s multiprocessing library). After all chunks are smoothed, they are merged back together. A progress bar shows the smoothing progress. After smoothing, it estimates normals for the points and saves the result to a new .ply file.
'''
import numpy as np
from scipy.spatial import KDTree
import open3d as o3d
from tqdm import tqdm  # 🆕 Progress bar
import multiprocessing as mp  # 🆕 For parallel processing

# 🆕 Function to smooth a chunk of points in parallel
def process_chunk(start_idx, end_idx, points, radius, smoothing_factor):
    tree = KDTree(points)
    chunk_smoothed = np.copy(points[start_idx:end_idx])
    
    for local_i, i in enumerate(range(start_idx, end_idx)):
        point = points[i]
        idx = tree.query_ball_point(point, radius)
        if len(idx) > 1:
            for idx_neighbor in idx:
                neighbor_position = chunk_smoothed[idx_neighbor - start_idx] if start_idx <= idx_neighbor < end_idx else points[idx_neighbor]
                direction = point - neighbor_position
                move_distance = np.linalg.norm(direction)
                direction_normalized = direction / move_distance if move_distance != 0 else direction
                if start_idx <= idx_neighbor < end_idx:
                    chunk_smoothed[idx_neighbor - start_idx] = neighbor_position + direction_normalized * smoothing_factor
    return start_idx, chunk_smoothed

# 🆕 Main smoothing function with multiprocessing and progress bar
def smooth_point_cloud_towards_center(pcd, radius=0.1, smoothing_factor=0.05, num_workers=mp.cpu_count()):
    points = np.asarray(pcd.points)
    num_points = len(points)
    chunk_size = num_points // num_workers
    ranges = [(i * chunk_size, (i + 1) * chunk_size if i != num_workers - 1 else num_points) for i in range(num_workers)]

    # 🆕 Multiprocessing
    with mp.Pool(processes=num_workers) as pool:
        results = list(tqdm(
            pool.starmap(process_chunk, [(start, end, points, radius, smoothing_factor) for start, end in ranges]),
            total=len(ranges),
            desc="Smoothing point cloud"
        ))

    # 🆕 Merge chunks back together
    smoothed_points = np.copy(points)
    for start_idx, chunk_data in results:
        smoothed_points[start_idx:start_idx + len(chunk_data)] = chunk_data

    # Return as new point cloud
    smoothed_pcd = o3d.geometry.PointCloud()
    smoothed_pcd.points = o3d.utility.Vector3dVector(smoothed_points)

    # 🆕 Estimate normals
    smoothed_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
    smoothed_pcd.normalize_normals()

    return smoothed_pcd

# === Main Execution ===

# Load point cloud
pcd = o3d.io.read_point_cloud(r"C:\Users\jtau4\Documents\lidar(relaxation)\barrows_pointcloud.bag_point_cloud.ply")

# Apply smoothing
smoothed_pcd = smooth_point_cloud_towards_center(
    pcd,
    radius=0.2,
    smoothing_factor=0.1,
    num_workers=mp.cpu_count()  # You can set this manually if you want
)

# Save result 🆕
o3d.io.write_point_cloud("smoothed_output.ply", smoothed_pcd)

# Optional: visualize
# o3d.visualization.draw_geometries([smoothed_pcd])