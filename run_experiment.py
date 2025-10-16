import os
import subprocess
import argparse
import time
import numpy as np
import open3d as o3d

# --- Configuration ---
# PROJECT_ROOT is the directory where this script is located.
PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

# --- USER ACTION REQUIRED ---
# Set this path to the root directory of your other project containing the Semantic3D data.
# If you leave this as None, the script will default to looking for a 'data' folder
# inside this current project directory.
#
# EXAMPLE:
# EXTERNAL_DATA_ROOT = "/Users/wangjiashu/Desktop/ComputerVision/RegionGrowing/data"
#
EXTERNAL_DATA_ROOT = "../../../../RegionGrowing/Implementation/data" # <-- SET YOUR PATH HERE

BUILD_DIR = os.path.join(PROJECT_ROOT, "build")
EXECUTABLE_PATH = os.path.join(BUILD_DIR, "region_merging")
RESULTS_DIR = os.path.join(PROJECT_ROOT, "results")
SCRIPTS_DIR = os.path.join(PROJECT_ROOT, "scripts")

def run_command(command):
    """Executes a command and raises an error if it fails."""
    print(f"\n> Running command: {' '.join(command)}")
    subprocess.run(command, check=True)

def transfer_labels(original_pcd_path, downsampled_pcd_path, downsampled_labels_path, output_labels_path):
    """
    Transfers labels from a downsampled cloud to the original dense cloud
    using a nearest neighbor search.
    """
    print("\n--- Starting Label Transfer (Upsampling) ---")
    
    print(f"Loading original point cloud from {original_pcd_path}...")
    original_points = np.loadtxt(original_pcd_path)[:, :3]
    pcd_original = o3d.geometry.PointCloud()
    pcd_original.points = o3d.utility.Vector3dVector(original_points)

    print(f"Loading downsampled point cloud from {downsampled_pcd_path}...")
    downsampled_points = np.loadtxt(downsampled_pcd_path)[:, :3]
    pcd_downsampled = o3d.geometry.PointCloud()
    pcd_downsampled.points = o3d.utility.Vector3dVector(downsampled_points)

    print(f"Loading predicted labels for downsampled cloud from {downsampled_labels_path}...")
    downsampled_labels = np.loadtxt(downsampled_labels_path, dtype=np.int32)
    
    if len(pcd_downsampled.points) != len(downsampled_labels):
        raise ValueError("Mismatch between downsampled points and their labels.")

    print("Building KDTree for nearest neighbor search...")
    kdtree = o3d.geometry.KDTreeFlann(pcd_downsampled)

    print(f"Finding nearest neighbors for {len(pcd_original.points)} points...")
    upsampled_labels = np.zeros(len(pcd_original.points), dtype=np.int32)
    
    for i, point in enumerate(pcd_original.points):
        if i % 100000 == 0 and i > 0:
             print(f"  Processing point {i}/{len(pcd_original.points)}...")
        [k, idx, _] = kdtree.search_knn_vector_3d(point, 1)
        if k > 0:
            upsampled_labels[i] = downsampled_labels[idx[0]]

    print(f"Saving upsampled labels to: {output_labels_path}")
    np.savetxt(output_labels_path, upsampled_labels, fmt='%d')
    print("--- Label Transfer Complete ---")
    return output_labels_path

def create_visualization_file(points_file, labels_file, output_file):
    """
    Combines a point cloud geometry file and a label file into a single
    file for easy visualization in CloudCompare.
    Format: X Y Z label
    """
    print(f"\n--- Creating Combined File for Visualization: {os.path.basename(output_file)} ---")
    try:
        print(f"Reading points from: {points_file}")
        points = np.loadtxt(points_file)[:, :3]
        
        print(f"Reading labels from: {labels_file}")
        labels = np.loadtxt(labels_file, dtype=np.int32).reshape(-1, 1)

        if points.shape[0] != labels.shape[0]:
            raise ValueError(f"Point count ({points.shape[0]}) and label count ({labels.shape[0]}) do not match!")

        combined_data = np.hstack((points, labels))
        
        np.savetxt(output_file, combined_data, fmt='%.6f %.6f %.6f %d')
        print(f"Successfully saved visualization file to: {output_file}")
    
    except Exception as e:
        print(f"Error creating visualization file: {e}")

def main(args):
    start_total_time = time.time()
    
    # --- MODIFIED: Determine the correct data directory from the hardcoded variable ---
    if EXTERNAL_DATA_ROOT:
        # Use the user-provided external data directory
        data_dir = os.path.abspath(EXTERNAL_DATA_ROOT)
        print(f"Using external data root: {data_dir}")
        if not os.path.exists(data_dir):
            print(f"Error: The hardcoded path in EXTERNAL_DATA_ROOT does not exist: {data_dir}")
            print("Please update the variable at the top of the script.")
            return
    else:
        # Default to the local 'data' directory within this project
        data_dir = os.path.join(PROJECT_ROOT, "data")
        print(f"Using local data directory: {data_dir}")

    # Define INPUT file paths using the determined data_dir
    raw_pcd_path = os.path.join(data_dir, "raw", "semantic3d", args.scene_name + ".txt")
    raw_labels_path = os.path.join(data_dir, "raw", "semantic3d", args.scene_name + ".labels")
    processed_dir = os.path.join(data_dir, "processed", "semantic3d")
    downsampled_pcd_path = os.path.join(processed_dir, args.scene_name + "_downsampled.txt")
    
    # Define OUTPUT file paths. These always point to the local results directory.
    os.makedirs(RESULTS_DIR, exist_ok=True)
    downsampled_pred_path = os.path.join(RESULTS_DIR, args.scene_name + "_downsampled_predicted.labels")
    final_pred_path = os.path.join(RESULTS_DIR, args.scene_name + "_final_predicted.labels")
    final_mapped_labels_path = os.path.join(RESULTS_DIR, args.scene_name + "_final_mapped.labels")
    raw_clusters_vis_path = os.path.join(RESULTS_DIR, args.scene_name + "_raw_clusters_visualization.txt")
    semantic_vis_path = os.path.join(RESULTS_DIR, args.scene_name + "_semantic_visualization.txt")

    if not args.eval_only:
        # --- 1. Pre-processing Step ---
        print("\n========================================")
        print("        STEP 1: PRE-PROCESSING")
        print("========================================")
        
        os.makedirs(processed_dir, exist_ok=True)
        
        preprocess_cmd = [
            "python", os.path.join(SCRIPTS_DIR, "preprocess.py"),
            raw_pcd_path, downsampled_pcd_path, "--voxel_size", "0.1"
        ]
        run_command(preprocess_cmd)
        
        # --- 2. Segmentation Step (C++) ---
        print("\n========================================")
        print("         STEP 2: SEGMENTATION")
        print("========================================")
        if not os.path.exists(EXECUTABLE_PATH):
            print(f"Error: Executable not found at {EXECUTABLE_PATH}. Please compile the C++ code first.")
            return
        
        segment_cmd = [EXECUTABLE_PATH, downsampled_pcd_path, downsampled_pred_path]
        run_command(segment_cmd)

    # --- 3. Label Transfer and Evaluation Step ---
    print("\n========================================")
    print("          STEP 3: EVALUATION")
    print("========================================")
    
    if not os.path.exists(downsampled_pred_path):
        print(f"Error: Predicted labels for downsampled cloud not found at {downsampled_pred_path}")
        print("Please run the full pipeline first (without --eval_only).")
        return
        
    transfer_labels(raw_pcd_path, downsampled_pcd_path, downsampled_pred_path, final_pred_path)
    
    evaluate_cmd = [
        "python", os.path.join(SCRIPTS_DIR, "evaluate.py"),
        raw_labels_path, final_pred_path,
        "--output_mapped_file", final_mapped_labels_path
    ]
    run_command(evaluate_cmd)

    # --- 4. (Optional) Create Visualization Files ---
    if not args.no_visualize:
        create_visualization_file(downsampled_pcd_path, downsampled_pred_path, raw_clusters_vis_path)

        if os.path.exists(final_mapped_labels_path):
            create_visualization_file(raw_pcd_path, final_mapped_labels_path, semantic_vis_path)
        else:
            print(f"Warning: Mapped labels file not found at {final_mapped_labels_path}. Skipping final visualization.")

    end_total_time = time.time()
    print(f"\nTotal execution time: {end_total_time - start_total_time:.2f} seconds.")

if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description="Run a full segmentation and evaluation experiment.")
                             
    parser.add_argument("--scene_name", type=str, 
                        default="marketsquarefeldkirch4-reduced",
                        help="The name of the Semantic3D scene to process.")
    parser.add_argument("--eval_only", action="store_true",
                        help="Skip pre-processing and segmentation, and run only evaluation on existing results.")
    parser.add_argument("--no-visualize", action="store_true",
                        help="Disable creating the combined visualization file (on by default).")
    
    args = parser.parse_args()
    main(args)

