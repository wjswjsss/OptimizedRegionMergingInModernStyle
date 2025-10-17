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
    print("========================================")
    
    # Print confusion matrix summary
    print("\nConfusion Matrix Summary:")
    print(f"  Total points: {conf_matrix.sum()}")
    print(f"  Correctly classified: {intersection.sum()} ({100*intersection.sum()/conf_matrix.sum():.2f}%)")
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
    
    # Statistics tracking
    num_noise_points = np.sum(predictions < 0)
    mixed_clusters = 0
    total_clusters = len(cluster_ids)
    purity_sum = 0.0
    
    print(f"  Found {total_clusters} clusters")
    print(f"  Noise/unassigned points: {num_noise_points} ({100*num_noise_points/len(predictions):.2f}%)")
    
    for cluster_id in cluster_ids:
        # Get the ground truth labels for all points in this cluster
        mask = predictions == cluster_id
        gt_in_cluster = ground_truth[mask]
        
        if len(gt_in_cluster) == 0:
            continue
        
        # Find the most frequent ground truth label in this cluster (TRUE majority vote)
        counts = np.bincount(gt_in_cluster, minlength=NUM_CLASSES)
        best_class = np.argmax(counts)
        
        # Calculate cluster purity (how "pure" is this cluster?)
        purity = counts[best_class] / len(gt_in_cluster)
        purity_sum += purity
        
        if purity < 0.8:  # Less than 80% of points belong to the majority class
            mixed_clusters += 1
        
        # Assign this semantic class to all points in the cluster
        mapped_predictions[mask] = best_class
    
    # Print statistics
    avg_purity = purity_sum / total_clusters if total_clusters > 0 else 0
    print(f"  Average cluster purity: {avg_purity:.2%}")
    print(f"  Mixed clusters (purity < 80%): {mixed_clusters}/{total_clusters} ({100*mixed_clusters/total_clusters:.1f}%)")
    
    # Check ground truth distribution
    unique_gt, counts_gt = np.unique(ground_truth, return_counts=True)
    print(f"\n  Ground Truth Distribution:")
    for cls, cnt in zip(unique_gt, counts_gt):
        print(f"    Class {cls} ({CLASS_NAMES[cls]}): {cnt} points ({100*cnt/len(ground_truth):.2f}%)")
    
    return mapped_predictions


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate segmentation results.")
    parser.add_argument("gt_file", type=str, help="Path to the ground truth .labels file.")
    parser.add_argument("pred_file", type=str, help="Path to the predicted (and upsampled) labels file.")
    parser.add_argument("--output_mapped_file", type=str, help="Path to save the final mapped class labels.")
    
    args = parser.parse_args()

    # Load labels
    print(f"Loading ground truth from: {args.gt_file}")
    gt_labels = np.loadtxt(args.gt_file, dtype=np.int32)
    
    print(f"Loading predictions from: {args.pred_file}")
    pred_labels = np.loadtxt(args.pred_file, dtype=np.int32)

    if len(gt_labels) != len(pred_labels):
        raise ValueError("Error: Ground truth and prediction files have different number of points!")
    
    print(f"Total points: {len(gt_labels)}")
    
    # Map cluster IDs to semantic class IDs using true majority voting
    mapped_pred_labels = map_clusters_to_classes(gt_labels, pred_labels)

    # Save mapped labels if requested
    if args.output_mapped_file:
        print(f"Saving final mapped semantic labels to: {args.output_mapped_file}")
        np.savetxt(args.output_mapped_file, mapped_pred_labels, fmt='%d')

    # Calculate mIoU with the mapped labels
    calculate_miou(gt_labels, mapped_pred_labels)