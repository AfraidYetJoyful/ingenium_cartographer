'''
Same as Multiprocessor1 but introduced a step_size variable, allowing you to skip every x points, speeding up the process but decreasing quality. Also introduces num_Workers which is the amount of cores that are being used. Multiprocessor1 used all cores available but that lagged my computer a lot and i was unable to do anything else on the computer, so this should allow a little more customizability to the process. 
'''
import numpy as np
from scipy.spatial import KDTree
import open3d as o3d
from tqdm import tqdm  #  Progress bar
import multiprocessing as mp  #  For parallel processing

#  Function to smooth a chunk of points in parallel
def process_chunk(start_idx, end_idx, points, radius, smoothing_factor, step_size):
    tree = KDTree(points)
    chunk_smoothed = np.copy(points[start_idx:end_idx])
    
    for local_i, i in enumerate(range(start_idx, end_idx, step_size)):
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

#  Main smoothing function with multiprocessing and progress bar
def smooth_point_cloud_towards_center(pcd, radius=0.1, smoothing_factor=0.05, step_size=10, num_workers=mp.cpu_count()):
    points = np.asarray(pcd.points)
    num_points = len(points)
    chunk_size = num_points // num_workers
    ranges = [(i * chunk_size, (i + 1) * chunk_size if i != num_workers - 1 else num_points) for i in range(num_workers)]

    with mp.Pool(processes=num_workers) as pool:
        results = list(tqdm(
            pool.starmap(process_chunk, [(start, end, points, radius, smoothing_factor, step_size) for start, end in ranges]),
            total=len(ranges),
            desc="Smoothing point cloud"
        ))

    smoothed_points = np.copy(points)
    for start_idx, chunk_data in results:
        smoothed_points[start_idx:start_idx + len(chunk_data)] = chunk_data

    smoothed_pcd = o3d.geometry.PointCloud()
    smoothed_pcd.points = o3d.utility.Vector3dVector(smoothed_points)
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
    step_size=100000,
    num_workers=6
)

# Save result
o3d.io.write_point_cloud("smoothed_output.ply", smoothed_pcd)

# Optional: visualize
# o3d.visualization.draw_geometries([smoothed_pcd])


