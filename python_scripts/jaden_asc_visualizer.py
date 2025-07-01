import open3d as o3d
import numpy as np

def load_asc(file_path):
    """ Load an .asc point cloud file (assuming X Y Z format). """
    points = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 3:  # Ensure at least X, Y, Z columns exist
                x, y, z = map(float, parts[:3])
                points.append([x, y, z])
    
    return np.array(points)

def visualize_point_cloud(file_path):
    """ Load and visualize the point cloud. """
    points = load_asc(file_path)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python visualize_asc.py <path_to_asc_file>")
        sys.exit(1)
    
    visualize_point_cloud(sys.argv[1])