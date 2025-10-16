import numpy as np
from sklearn.metrics import confusion_matrix
import argparse

# Define the class mapping for Semantic3D reduced-8
# Class 0 is 'unlabeled'
CLASS_NAMES = [
    "unlabeled", "man-made terrain", "natural terrain", "high vegetation",
    "low vegetation", "buildings", "hard scape", "scanning artefacts", "cars"
]
NUM_CLASSES = len(CLASS_NAMES)

def calculate_miou(ground_truth_labels, predicted_labels):
    """
    Calculates the Mean Intersection over Union (mIoU) score.
    """
    print("Calculating mIoU...")
    
    # Ensure labels are integers
    ground_truth_labels = ground_truth_labels.astype(np.int64)
    predicted_labels = predicted_labels.astype(np.int64)
    
    # Compute the confusion matrix
    conf_matrix = confusion_matrix(
        ground_truth_labels, 
        predicted_labels, 
        labels=np.arange(NUM_CLASSES)
    )
    
    # Calculate Intersection and Union from the matrix
    intersection = np.diag(conf_matrix)
    ground_truth_set = conf_matrix.sum(axis=1)
    predicted_set = conf_matrix.sum(axis=0)
    union = ground_truth_set + predicted_set - intersection
    
    # Calculate IoU for each class, avoiding division by zero
    iou = np.divide(
        intersection, union, 
        out=np.zeros_like(intersection, dtype=float), 
        where=union != 0
    )
    
    # The official benchmark evaluates on classes 1 through 8
    # We ignore class 0 (unlabeled)
    valid_classes_iou = iou[1:]
    mean_iou = np.mean(valid_classes_iou)
    
    # --- Print Results ---
    print("\n========================================")
    print("           Evaluation Results")
    print("========================================")
    print(f"Mean IoU (mIoU): {mean_iou:.4f}")
    print("----------------------------------------")
    print("Per-Class IoU:")
    for i in range(1, NUM_CLASSES):
        print(f"  - {CLASS_NAMES[i]:<20}: {iou[i]:.4f}")
    print("========================================\n")
    
    return mean_iou, iou

def map_clusters_to_classes(ground_truth, predictions):
    """
    Assigns each cluster from region growing to the semantic class
    it overlaps with the most (majority voting).
    """
    print("Mapping unsupervised clusters to semantic classes...")
    
    # Find unique cluster IDs from the prediction (e.g., 0, 1, 2, ...)
    # We ignore negative labels which represent noise or unlabeled points
    cluster_ids = np.unique(predictions[predictions >= 0])
    
    # Create a new array for the mapped predictions
    mapped_predictions = np.zeros_like(predictions)
    
    for cluster_id in cluster_ids:
        # Get the ground truth labels for all points in this cluster
        gt_in_cluster = ground_truth[predictions == cluster_id]
        
        if len(gt_in_cluster) == 0:
            continue
            
        # Find the most frequent ground truth label in this cluster (majority vote)
        # We ignore 'unlabeled' (class 0) if possible
        counts = np.bincount(gt_in_cluster)
        
        best_class = 0
        if len(counts) > 1 and np.sum(counts[1:]) > 0:
            # Prefer a semantic class over 'unlabeled'
            best_class = np.argmax(counts[1:]) + 1
        elif len(counts) > 0:
            # Fallback to most frequent if only 'unlabeled' is present
            best_class = np.argmax(counts)
            
        # Assign this semantic class to all points in the cluster
        mapped_predictions[predictions == cluster_id] = best_class
        
    return mapped_predictions


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate segmentation results.")
    parser.add_argument("gt_file", type=str, help="Path to the ground truth .labels file.")
    parser.add_argument("pred_file", type=str, help="Path to the predicted (and upsampled) labels file.")
    # --- ADDED ---
    # Add an optional argument to save the mapped labels for visualization
    parser.add_argument("--output_mapped_file", type=str, help="Path to save the final mapped class labels.")
    
    args = parser.parse_args()

    # Load labels
    print(f"Loading ground truth from: {args.gt_file}")
    gt_labels = np.loadtxt(args.gt_file, dtype=np.int32)
    
    print(f"Loading predictions from: {args.pred_file}")
    pred_labels = np.loadtxt(args.pred_file, dtype=np.int32)

    if len(gt_labels) != len(pred_labels):
        raise ValueError("Error: Ground truth and prediction files have different number of points!")
    
    # --- IMPORTANT STEP ---
    # Since the C++ code is unsupervised, we must first map its cluster IDs
    # (0, 1, 2...) to the ground truth class IDs (0-8)
    mapped_pred_labels = map_clusters_to_classes(gt_labels, pred_labels)

    # --- ADDED ---
    # If the output path is provided, save the mapped labels
    if args.output_mapped_file:
        print(f"Saving final mapped semantic labels to: {args.output_mapped_file}")
        np.savetxt(args.output_mapped_file, mapped_pred_labels, fmt='%d')

    # Now, calculate mIoU with the mapped labels
    calculate_miou(gt_labels, mapped_pred_labels)
