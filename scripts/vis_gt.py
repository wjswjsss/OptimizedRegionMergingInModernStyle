import numpy as np
import os

def create_gt_visualization_file():
    """
    Combines a raw point cloud file with its ground truth labels file
    to create a single file for easy visualization in CloudCompare.
    """
    print("========================================")
    print("Creating Ground Truth Visualization File")
    print("========================================")

    # --- Configuration ---
    # Assumes this script is in the 'scripts/' directory.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.join(script_dir, '..')

    # Define the input files (the original, raw data)
    points_filename = "marketsquarefeldkirch4-reduced.txt"
    labels_filename = "marketsquarefeldkirch4-reduced.labels"

    points_file_path = os.path.join(project_root, 'data', 'raw', 'semantic3d', points_filename)
    labels_file_path = os.path.join(project_root, 'data', 'raw', 'semantic3d', labels_filename)

    # Define the output file path
    output_filename = points_filename.replace('.txt', '_ground_truth_visualization.txt')
    output_file_path = os.path.join(project_root, 'results', output_filename)
    
    # Create results directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

    # --- File Processing ---
    try:
        print(f"Loading points from: {points_file_path}")
        # Loads X, Y, Z, Intensity, R, G, B
        points_data = np.loadtxt(points_file_path)

        print(f"Loading ground truth labels from: {labels_file_path}")
        labels_data = np.loadtxt(labels_file_path, dtype=np.int32)

        # Ensure the number of points and labels match
        if len(points_data) != len(labels_data):
            print("\nError: The number of points and labels do not match!")
            print(f"  - Points found: {len(points_data)}")
            print(f"  - Labels found: {len(labels_data)}")
            return

        print("Combining points and labels...")
        # Add the labels as a new column to the points data
        # Reshape labels to be a column vector for stacking
        combined_data = np.column_stack((points_data, labels_data.reshape(-1, 1)))

        print(f"Saving combined file to: {output_file_path}")
        # Save with space delimiter. We use a mixed format string to handle floats and the final integer label.
        np.savetxt(output_file_path, combined_data, fmt='%f %f %f %d %d %d %d %d')
        
        print("\nSuccess! The ground truth visualization file has been created.")

    except FileNotFoundError as e:
        print(f"\nError: File not found! Please check your paths.")
        print(e)
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")


if __name__ == '__main__':
    create_gt_visualization_file()