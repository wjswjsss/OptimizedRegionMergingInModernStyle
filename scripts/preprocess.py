import open3d as o3d
import numpy as np
import os
import argparse

def downsample_point_cloud(input_path, output_path, voxel_size):
    """
    Loads a Semantic3D .txt file, performs voxel downsampling,
    and saves the result back to a .txt file.
    """
    print(f"Loading point cloud from: {input_path}")
    
    # Load the data using numpy (faster for large text files)
    try:
        data = np.loadtxt(input_path)
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    # Extract XYZ coordinates
    points = data[:, :3]
    
    # Create an Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    print(f"Original number of points: {len(pcd.points)}")
    
    # Perform voxel downsampling
    downsampled_pcd = pcd.voxel_down_sample(voxel_size)
    
    print(f"Downsampled to {len(downsampled_pcd.points)} points with voxel size {voxel_size}")
    
    # Save the downsampled points to a new .txt file
    np.savetxt(output_path, np.asarray(downsampled_pcd.points), fmt='%.6f')
    
    print(f"Successfully saved downsampled cloud to: {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Downsample a Semantic3D point cloud.")
    parser.add_argument("input_file", type=str, help="Path to the input .txt point cloud file.")
    parser.add_argument("output_file", type=str, help="Path to save the downsampled .txt file.")
    parser.add_argument("--voxel_size", type=float, default=0.1, help="Voxel size for downsampling (e.g., 0.1 meters).")
    
    args = parser.parse_args()
    
    # Ensure the output directory exists
    output_dir = os.path.dirname(args.output_file)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        
    downsample_point_cloud(args.input_file, args.output_file, args.voxel_size)
